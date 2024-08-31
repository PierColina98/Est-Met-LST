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

/* @file AHT21.h
 * @brief A library for communicating with the AHT21 sensor using ESP-IDF.
 *
 * Dependencies:
 * - ESP-IDF for ESP32
 *
 *
 * @author Pier Colina
 * @date 02/07/2024
 * @version 1.0
 *
 */
#ifndef AHT21_H
#define AHT21_H

#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"

// AHT21 REGISTERS AND COMMANDS
#define STATUS_CMD 0x71
#define OK_STATUS 0x18
#define TRIGGER_MEAS 0xAC
#define MEAS_CMD1 0X33
#define MEAS_CMD2 0X00

/// @brief Initialize the library. Configures the I2C master bus and adds the AHT21 I2C device address to the bus. 
///        If I2C bus and device are configured correctly, sends the status command to the AHT21 and verifies communication with the sensor.
/// @return ESP_OK on success. ESP_ERR on failure.
esp_err_t AHT21_init();

/// @brief Reads temperature and humidity data from the sensor.
/// @param temp Pointer to the float variable to store the temperature value.
/// @param hum Pointer to the float variable to store the humidity value.
/// @return ESP_OK on success. ESP_ERR on failure.
esp_err_t AHT21_get_data(float *temp, float *hum);

/// @brief Deinitialize the library. Removes the I2C device and recycles the I2C master bus.
/// @return ESP_OK on success. ESP_ERR on failure.
esp_err_t AHT21_deinit();

#endif