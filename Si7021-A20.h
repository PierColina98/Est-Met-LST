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

/* @file Si7021-A20.h
 * @brief A single file library for communicating with the Si7021-A20 sensor using ESP-IDF.
 *
 * This library aims to provide an interface to initialize, configure, and read data from the Si7021
 * humidity and temperature sensor using the ESP32 microcontroller and the ESP-IDFv5.4.0. The Si7021 offers 
 * an accurate digital solution ideal for measuring temperature, humidity and dew-point.
 * 
 * Dependencies:
 * - ESP-IDFv5.4.0 or higher
 *
 *
 * @author Pier Colina
 * @date 30/08/2024
 * @version 1.0
 *
 */
#pragma once

#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"

// Si7021 I2C 7-bit Device Address
#define Si7021_DEV_ADD 0x40

// Si7021 Command Codes
#define RH_H_CMD 0xE5
#define RH_NH_CMD 0xF5
#define T_H_CMD 0xE3
#define T_NH_CMD 0xF3
#define T_RH_CMD 0xE0
#define RESET_CMD 0xFE
#define USER_REG_W_CMD 0xE6
#define USER_REG_R_CMD 0xE7
#define HTRE_REG_W_CMD 0xE6
#define HTRE_REG_R_CMD 0xE6

typedef enum
{
    RES_0,
    RES_1,
    RES_2,
    RES_3
} Si7021_RES;

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint16_t Raw_Temperature;
    uint16_t Raw_Humidity;
} Si7021_DATA;

esp_err_t Si7021_init(Si7021_RES resolution);
esp_err_t Si7021_get_data(float *temp, float *hum);
esp_err_t Si7021_deinit();
