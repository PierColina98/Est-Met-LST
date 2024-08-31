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

#include "Si7021-A20.h"

void Si7021_read_user_reg(uint8_t *write_dat);
void Si7021_write_user_reg(uint8_t *write_dat);

Si7021_DATA si7021data = {0};

esp_err_t Si7021_init(Si7021_RES resolution)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = Si7021_DEV_ADD,
        .scl_speed_hz = 100000,
    };

    if (i2c_new_master_bus(&bus_cfg, &si7021data.bus_handle) != ESP_OK)
    {
        ESP_LOGI("I2C", "MASTER BUS ERROR.");
        return ESP_ERR_NOT_FINISHED;
    }

    if (i2c_master_probe(si7021data.bus_handle, Si7021_DEV_ADD, -1) != ESP_OK)
    {
        ESP_LOGI("I2C", "DEV ADDR NOT FOUND.");
        i2c_del_master_bus(si7021data.bus_handle);
        return ESP_ERR_NOT_FINISHED;
    }
    else
    {
        if (i2c_master_bus_add_device(si7021data.bus_handle, &dev_cfg, &si7021data.dev_handle) != ESP_OK)
        {
            ESP_LOGI("I2C", "DEV HANDLER ERROR.");
            i2c_del_master_bus(si7021data.bus_handle);
            return ESP_ERR_NOT_FINISHED;
        }
    }

    uint8_t write_data;
    Si7021_read_user_reg(&write_data);
    printf("%d\n", write_data);
    switch (resolution)
    {
    case RES_0:
        write_data |= 0x00;
        break;
    case RES_1:
        write_data |= 0x01;
        break;
    case RES_2:
        write_data |= 0x80;
        break;
    case RES_3:
        write_data |= 0x81;
        break;
    
    default:
        write_data |= 0x00;
        break;
    }
    Si7021_write_user_reg(&write_data);
    return ESP_OK;
};

esp_err_t Si7021_deinit()
{
    if(i2c_master_bus_rm_device(si7021data.dev_handle) != ESP_OK){
        ESP_LOGI("I2C", "DEV HANDLE REMOVE ERROR.");
        return ESP_ERR_NOT_FINISHED;
    }else{
        if (i2c_del_master_bus(si7021data.bus_handle) != ESP_OK){
            ESP_LOGI("I2C", "BUS HANDLE REMOVE ERROR.");
            return ESP_ERR_NOT_FINISHED;
        }else{
            return ESP_OK;
        }
    }
};

esp_err_t Si7021_get_data(float *temp, float *hum){
    // Relative Humidity
    uint8_t read_cmd = RH_NH_CMD;
    uint8_t read_buffer[3];
    uint16_t raw_data;
    ESP_ERROR_CHECK(i2c_master_transmit(si7021data.dev_handle, &read_cmd, sizeof(uint8_t), -1));
    vTaskDelay(30/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(si7021data.dev_handle, read_buffer, sizeof(uint8_t) * 3, -1));
    raw_data = (read_buffer[0] << 8) | read_buffer[1];
    *hum = (125.0 * raw_data / 65536.0) - 6;
    // Temperature
    read_cmd = T_NH_CMD;
    ESP_ERROR_CHECK(i2c_master_transmit(si7021data.dev_handle, &read_cmd, sizeof(uint8_t), -1));
    vTaskDelay(30/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(si7021data.dev_handle, read_buffer, sizeof(uint8_t) * 3, -1));
    raw_data = (read_buffer[0] << 8) | read_buffer[1];
    *temp = (175.72 * raw_data / 65536.0) - 46.85;
    return ESP_OK;
};

void Si7021_read_user_reg(uint8_t *write_dat){
    uint8_t read_cmd = USER_REG_R_CMD;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(si7021data.dev_handle, &read_cmd, sizeof(uint8_t), write_dat, sizeof(uint8_t), -1));
};

void Si7021_write_user_reg(uint8_t *write_dat){
    uint8_t write_cmd = USER_REG_W_CMD;
    i2c_master_transmit_multi_buffer_info_t cmd[2] = {
        {.write_buffer = &write_cmd, .buffer_size = 1},
        {.write_buffer = write_dat, .buffer_size = 1}};
    ESP_ERROR_CHECK(i2c_master_multi_buffer_transmit(si7021data.dev_handle, cmd, sizeof(cmd) / sizeof(i2c_master_transmit_multi_buffer_info_t), -1));
};