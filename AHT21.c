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

#include "AHT21.h"

i2c_master_bus_handle_t bus_hdl;
i2c_master_dev_handle_t dev_hdl;

uint8_t AHT21_check_status();

esp_err_t AHT21_init()
{
    esp_err_t err;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = false,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .intr_priority = 0,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21};

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x38,
        .scl_speed_hz = 100000};

    err = i2c_new_master_bus(&bus_cfg, &bus_hdl);
    if (err != ESP_OK)
    {
        ESP_LOGI("I2C", "BUS HANDLER ERROR.");
        return err;
    }
    else
    {
        err = i2c_master_bus_add_device(bus_hdl, &dev_cfg, &dev_hdl);
        if (err != ESP_OK)
        {
            ESP_LOGI("I2C", "DEV HANDLER ERROR.");
            i2c_del_master_bus(bus_hdl);
            return err;
        }
    }

    uint8_t status = AHT21_check_status();
    if (status != 0x18)
    {
        return ESP_ERR_NOT_FINISHED;
    }else{
        return ESP_OK;
    }
};

uint8_t AHT21_check_status()
{
    esp_err_t err;
    uint8_t write_cmd = STATUS_CMD;
    uint8_t read_buffer = 0;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    err = i2c_master_transmit_receive(dev_hdl, &write_cmd, sizeof(uint8_t), &read_buffer, sizeof(uint8_t), -1);
    if (err != ESP_OK)
    {
        return 0x00;
    }
    else
    {
        return read_buffer;
    }
}

esp_err_t AHT21_get_data(float *temp, float *hum)
{
    esp_err_t err;
    uint8_t meas_cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t read_buff[7];
    int32_t raw_temp, raw_hum;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    err = i2c_master_transmit(dev_hdl, meas_cmd, sizeof(meas_cmd), -1);
    if (err != ESP_OK)
    {
        return err;
    }
    vTaskDelay(80 / portTICK_PERIOD_MS);
    err = i2c_master_receive(dev_hdl, read_buff, sizeof(read_buff), -1);
    if (err != ESP_OK)
    {
        return err;
    }
    else
    {
        puts("RAW DATA: ");
        for (int i = 0; i < sizeof(read_buff); i++)
        {
            printf("%x\n", read_buff[i]);
        }
        puts("END OF RAW DATA.");
        raw_hum = (read_buff[1] << 12) | (read_buff[2] << 4) | ((read_buff[3] >> 4) & 0x0F);
        raw_temp = ((read_buff[3] & 0x0F) << 16) | (read_buff[4] << 8) | read_buff[5];
        puts("VALUES: ");
        printf("temperature: %.1f\n", (raw_temp / 1048576.0) * 200.0 - 50.0);
        printf("humidity: %.1f\n", (raw_hum / 1048576.0) * 100.0);
        *temp = (raw_temp / 1048576.0) * 200.0 - 50.0;
        *hum = (raw_hum / 1048576.0) * 100;
        return ESP_OK;
    }
};

esp_err_t AHT21_deinit(){
    if(i2c_master_bus_rm_device(dev_hdl) != ESP_OK){
        ESP_LOGI("I2C", "DEV HANDLE REMOVE ERROR.");
        return ESP_ERR_NOT_FINISHED;
    }else{
        if (i2c_del_master_bus(bus_hdl) != ESP_OK){
            ESP_LOGI("I2C", "BUS HANDLE REMOVE ERROR.");
            return ESP_ERR_NOT_FINISHED;
        }else{
            return ESP_OK;
        }
    }
};