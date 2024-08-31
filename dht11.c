#include "dht11.h"
gpio_num_t dht_pin;
esp_err_t dht11_init(gpio_num_t pin)
{
    dht_pin = pin;
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,       // Disable interrupt
        .mode = GPIO_MODE_OUTPUT_OD,             // Set as output mode
        .pin_bit_mask = (1ULL << 15), // Bit mask for GPIO 4
        };
    esp_err_t msg = gpio_config(&io_conf);
    if (msg != ESP_OK){
        ESP_LOGI("DHT INIT", "Failed to configure GPIO PIN");
        return msg;
    };
    msg = gpio_set_level(pin, 1);
    if (msg != ESP_OK){
        ESP_LOGI("DHT INIT", "Failed to configure GPIO PIN");
        return msg;
    };
    return ESP_OK;
};

esp_err_t dht11_get_temphum(float *humidity, float *temperature){
    esp_err_t msg;
    //uint8_t data[5];
    /*
     * Initialize the communication by sending out the start signal:
     *       __           __________________                         _______________                                ____________________
     * GPIO    \  Start  / Pull-up and wait \  Response ACK signal  /  DHT pull-up  \   Start transmit 1-bit data  /    Data(1-bit)     \   Start transmit 1-bit data 
     *          \_______/                    \_____________________/                 \____________________________/                      \____ . . . .
     *          |  18ms |      20-40 us      |        80us         |       80us      |           50us             |   26-28us OR 70us     |         50us
     */
    msg = gpio_set_level(dht_pin, 0);
    if (msg != ESP_OK){
        ESP_LOGI("GPIO", "ERROR SET DIRECTION 0: %x", msg);
        return msg;
    };
    vTaskDelay(18/portTICK_PERIOD_MS);
    msg = gpio_set_level(dht_pin, 1);
    if (msg != ESP_OK){
        ESP_LOGI("GPIO", "ERROR SET DIRECTION 1: %x", msg);
        return msg;
    };
    ets_delay_us(40);
    if (gpio_get_level(dht_pin)){
        ESP_LOGI("DHT", "ACK FAILED.");
        return ESP_ERR_NOT_FINISHED;
    }else{
        ets_delay_us(80);
        ESP_LOGI("DHT", "ACK!");
        if(gpio_get_level(dht_pin)){
            ESP_LOGI("DHT", "DEV PULL-UP!");
            ets_delay_us(80);
        }else{
            ESP_LOGI("DHT", "DEV PULL-UP FAILED.");
            return ESP_ERR_NOT_FINISHED;
        }
    };
    for(int i = 0; i < DATA_TRANSF_SIZE; i++){
        for(int j = 0; j < 8; j++){
            int count = 0;
            ets_delay_us(50);
            while(gpio_get_level(dht_pin) == 1){
                count++;
            }
            printf("COUNT: %d", count);
        }
    }
    return ESP_OK;
};