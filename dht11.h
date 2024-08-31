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

/* @file dht11.h
 * @brief A library for communicating with the DHT11 sensor using ESP-IDF.
 *
 * Dependencies:
 * - ESP-IDF for ESP32
 *
 *
 * @author Pier Colina
 * @date 17/07/2024
 * @version 1.0
 *
 */
#ifndef DHT11_H
#define DHT11_H

#pragma once

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#define CONFIG_LOG_MAXIMUM_LEVEL 3
#define DATA_TRANSF_SIZE 5 // 5 BYTES = 40 BITS


esp_err_t dht11_init(gpio_num_t pin);

esp_err_t dht11_get_temphum(float *humidity, float *temperature);

#endif