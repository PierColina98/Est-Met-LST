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