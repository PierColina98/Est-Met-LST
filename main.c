/* GNU GENERAL PUBLIC LICENSE
 * Version 3, 29 June 2007
 *
 * Copyright (C) 2024 Pier Colina
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "mqtt5_client.h"
#include "cJSON.h"
#include "BMP180.h"
#include "Si7021-A20.h"
#include <math.h>

#define WIFI_SSID "ENTER-SSID"
#define WIFI_PASSWD "ENTER-PASSWD"
#define WAKEUP_TIME_SEC 300

/* Variables atmosféricas a medir:
 * Temperatura I2C
 * Humedad I2C
 * Presión barométrica I2C
 * Velocidad del viento GPIO
 * Dirección del viento ADC
 * Precipitación GPIO
 * Radiación solar ADC
 */

typedef struct
{
    float pressure;
    float temperature;
    float humidity;
    float altitude;
    float precipitacion;
    float wind_velocity;
    float wind_direction;
    char *current_time;
} sensor_data;

// FreeRTOS Tasks
void vReadPress_Temp_Hum(void *pvParameters);
void vReadWindDirection(void *pvParameters);
void vReadTipCount_Precip(void *pvParameters);
void vReadSpinCount_WindSpeed(void *pvParameters);

// Variables de control de estado
bool WifiConnected = false;
bool PressCheck = false;
bool TempHumCheck = false;
bool AltitudeCheck = false;
bool SpeedCheck = false;
bool DirCheck = false;
bool RainCheck = false;
bool MQTT_Conn = false;
bool MQTT_Ack = false;

// ESTRUCTURA DE DATOS
sensor_data *data;

// JSON Object
char *jsonString;

// Tasks Handlers
TaskHandle_t xPressureHandle = NULL;
TaskHandle_t xDirHandle = NULL;
TaskHandle_t xTipCountHandle = NULL;
TaskHandle_t xSpinCountHandle = NULL;

//WiFi reconnect retry
static int s_retry_num = 0;

// Declaración de funciones
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
char *createJSON_Object(sensor_data *data);
char *get_time();
int64_t timePassed(int64_t since);

void app_main()
{
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                        WAKE UP FROM DEEP SLEEP
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */
    // Retrieve MCU power-on reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI("WAKEUP", "Wake up from timer");
        break;
    default:
        ESP_LOGI("WAKEUP", "Not a deep sleep reset");
        break;
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                              NVS INIT
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */
    // Initialize NVS necessary for Wi-Fi and store Wi-Fi credentials
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // NVS handle
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("wifi-storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI("NVS", "Failed to open partition.");
        esp_restart();
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                        SENSOR PHASE (FREERTOS TASKS)
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */
    // Allocate memory for sensor_data
    data = malloc(sizeof(sensor_data));

    // Create sensor tasks
    xTaskCreatePinnedToCore(vReadPress_Temp_Hum, "PRESSURE_TEMP_HUM", 4096, (void *)data, 5, &xPressureHandle, 1);
    xTaskCreatePinnedToCore(vReadWindDirection, "WIND_DIR", 2048, (void *)data, 4, &xDirHandle, 1);
    xTaskCreatePinnedToCore(vReadTipCount_Precip, "PRECIPITATION", 4096, (void *)data, 3, &xTipCountHandle, 1);
    xTaskCreatePinnedToCore(vReadSpinCount_WindSpeed, "WIND_SPEED", 2048, (void *)data, 2, &xSpinCountHandle, 1);

    int64_t lastSensed = esp_timer_get_time() / 1000;
    while (!RainCheck || !DirCheck || !SpeedCheck || !PressCheck || !TempHumCheck || !AltitudeCheck)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (timePassed(lastSensed) > 10000)
        {
            esp_restart();
        }
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                           Wi-Fi LIBRARY INIT
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */
    // Initialize the TCP/IP stack
    esp_netif_init();
    // Create event loop task
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create a STA
    esp_netif_create_default_wifi_sta();
    // Default Wi-Fi configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // Initialize Wi-Fi library
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // Register Wi-Fi events (Only 2 necessary)
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    // Set SSID and Password
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    lastSensed = esp_timer_get_time() / 1000;
    while (!WifiConnected)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (timePassed(lastSensed) > 10000)
        {
            esp_restart();
        }
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                  MQTT LIBRARY INIT AND DATA TRANSMISSION
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */
    // Get datetime for MQTT message timestamp
    data->current_time = get_time();
    jsonString = createJSON_Object(data);
    // MQTT client init
    const esp_mqtt_client_config_t mqtt_cfg = {
        .buffer.out_size = 2048,
        // Public MQTT Broker
        .broker.address.hostname = "broker.hivemq.com",
        .broker.address.port = 1883,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .task.priority = 6,
        .session.last_will.topic = "/EST_MET/status",
        .session.last_will.msg = "MQTT Disconnected",
        .session.last_will.msg_len = 0,
        .session.last_will.qos = 2,
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1};
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    lastSensed = esp_timer_get_time() / 1000;
    while (!MQTT_Ack)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (timePassed(lastSensed) > 10000)
        {
            esp_restart();
        }
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                              DEEP SLEEP
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     */
    nvs_close(my_handle);
    free(data);
    esp_sleep_enable_timer_wakeup(WAKEUP_TIME_SEC * 1000000);
    esp_deep_sleep_start();
}

char *get_time()
{
    // Set the TZ environment variable to GMT-5
    setenv("TZ", "GMT0BST5", 1);
    // Configure NTP server to poll
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    // Initialize NTP library and request time.
    esp_netif_sntp_init(&config);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK)
    {
        printf("Failed to update system time within 10s timeout");
    }
    // Update C library runtime data for the new timezone.
    tzset();

    // Get the correct local time
    time_t now = time(NULL);
    struct tm *local_time = localtime(&now);
    // Return the local time as a human-readable string.
    return asctime(local_time);
};

char *createJSON_Object(sensor_data *data)
{
    // Root CJSON Object
    cJSON *sensorD = cJSON_CreateObject();
    // Sensor Data Values Array
    cJSON *values_array = cJSON_CreateArray();
    // Sensor Data Values
    // Pressure
    cJSON *pressure_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(pressure_json, "pressure", data->pressure);
    cJSON_AddItemReferenceToArray(values_array, pressure_json);
    // Temperature
    cJSON *temperature_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(temperature_json, "temperature", data->temperature);
    cJSON_AddItemReferenceToArray(values_array, temperature_json);
    // Humidity
    cJSON *humidity_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(humidity_json, "humidity", data->humidity);
    cJSON_AddItemReferenceToArray(values_array, humidity_json);
    // Altitude
    cJSON *altitude_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(altitude_json, "altitude", data->altitude);
    cJSON_AddItemReferenceToArray(values_array, altitude_json);
    // Precipitation
    cJSON *precipitation_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(precipitation_json, "precipitation", data->precipitacion);
    cJSON_AddItemReferenceToArray(values_array, precipitation_json);
    // Wind Velocity
    cJSON *velocity_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(velocity_json, "velocity", data->wind_velocity);
    cJSON_AddItemReferenceToArray(values_array, velocity_json);
    // Wind Direction
    cJSON *direction_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(direction_json, "direction", data->wind_direction);
    cJSON_AddItemReferenceToArray(values_array, direction_json);
    // Add array object to root JSON object
    cJSON_AddItemToObject(sensorD, "data", values_array);
    // Add timestamp to root JSON object
    cJSON_AddStringToObject(sensorD, "timestamp", data->current_time);
    // JSON to String
    char *string = cJSON_Print(sensorD);
    // Free resources
    cJSON_Delete(sensorD);

    return string;
};

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI("WIFI", "Connected to the AP");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        WifiConnected = false;
        if (s_retry_num < 5)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("WIFI", "retry to connect to the AP");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WIFI", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        WifiConnected = true;
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI("MQTT", "MQTT_EVENT_BEFORE_CONNECT: Initialized and ready to connect to broker.");
        break;
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
        MQTT_Conn = true;
        msg_id = esp_mqtt_client_publish(client, "/EST_MET/sensor_data", jsonString, 0, 2, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_DISCONNECTED");
        MQTT_Conn = false;
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        MQTT_Ack = true;
        esp_mqtt_client_disconnect(client);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTT", "MQTT_EVENT_ERROR");
        esp_mqtt_client_disconnect(client);
        break;
    default:
        ESP_LOGI("MQTT", "Other event id:%d, %d, %d", event->event_id, event->error_handle->connect_return_code, event->error_handle->error_type);
        break;
    }
}

void vReadPress_Temp_Hum(void *pvParameters)
{
    sensor_data *d = (sensor_data *)pvParameters;
    if (BMP180_init(3) == ESP_OK)
    {
        BMP180_get_press(&d->pressure);
        PressCheck = true;
        BMP180_deinit();
    }
    else
    {
        ESP_LOGI("BMP", "LIBRARY INIT FAILED.");
    }
    if (Si7021_init(RES_0) == ESP_OK)
    {
        Si7021_get_data(&d->temperature, &d->humidity);
        TempHumCheck = true;
        Si7021_deinit();
    }
    else
    {
        ESP_LOGI("Si7021", "LIBRARY INIT FAILED.");
    }
    d->altitude = 44330 * (1 - pow(d->pressure / 1013.25, 0.1903));
    AltitudeCheck = true;

    vTaskDelete(NULL);
}

void vReadWindDirection(void *pvParameters)
{
    sensor_data *d = (sensor_data *)pvParameters;
    int analogValueDir;
    // Due to ESP32 hardware limitations, ADC2 is shared with Wi-Fi driver. Therefore, the adc function must be called first in the code
    // and recycle the ADC-unit once the data has been read.
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .clk_src = 0,
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_10,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &analogValueDir));
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

    float wind_dir_angle = (analogValueDir * 0.3503) + 180;
    if (wind_dir_angle >= 360)
    {
        wind_dir_angle -= 360;
    }
    d->wind_direction = wind_dir_angle;
    DirCheck = true;

    vTaskDelete(NULL);
};

int64_t timePassed(int64_t since)
{
    int64_t now = esp_timer_get_time() / 1000;
    return now - since;
};

void vReadTipCount_Precip(void *pvParameters)
{
    sensor_data *d = (sensor_data *)pvParameters;
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << 17),
    };
    gpio_config(&io_conf);

    int tip_count = 0;
    float inch_per_tip = 0.01;
    int64_t lastOn = esp_timer_get_time() / 1000;
    while (timePassed(lastOn) < 5000)
    {
        if (gpio_get_level(GPIO_NUM_17) == 1)
        {
            tip_count++;
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
    if(tip_count >= 40){
        tip_count = 0;
    }
    float inches_in_duration = tip_count * inch_per_tip;
    float inches_per_sec = inches_in_duration / 5;
    d->precipitacion = inches_per_sec;
    RainCheck = true;
    printf("Rainfall rate: %.2f inches/sec\n", inches_per_sec);

    vTaskDelete(NULL);
};

void vReadSpinCount_WindSpeed(void *pvParameters)
{
    sensor_data *d = (sensor_data *)pvParameters;
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, 
        .mode = GPIO_MODE_INPUT,        
        .pin_bit_mask = (1ULL << 15),   
    };
    gpio_config(&io_conf);

    int rotation_count = 0;
    float calibration_factor = 0.45; // mph
    int64_t lastOn = esp_timer_get_time() / 1000;
    while (timePassed(lastOn) < 5000)
    {
        if (gpio_get_level(GPIO_NUM_15) == 0)
        {
            rotation_count++;
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
    if (rotation_count > 800)
    {
        rotation_count = 0;
    }
    float wind_speed = rotation_count * calibration_factor;
    d->wind_velocity = wind_speed;
    SpeedCheck = true;
    printf("Wind Speed: %.2f miles/hour\n", wind_speed);

    vTaskDelete(NULL);
};
