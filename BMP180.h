/* @file BMP180.h
 * @brief A single file library for communicating with the BMP180 sensor using ESP-IDF.
 *
 * This library provides an interface to initialize, configure, and read data from the BMP180
 * barometric pressure sensor using the ESP32 microcontroller and the ESP-IDFv5.4.0. The BMP180 sensor can measure
 * temperature and pressure, which can be used to calculate altitude.
 *
 * Features:
 * - Initialize the BMP180 sensor
 * - Set over sampling settings
 * - Read temperature and pressure values
 * - Calculate altitude based on pressure readings
 * - Easy-to-use API for seamless integration
 *
 * Dependencies:
 * - ESP-IDF for ESP32
 *
 * Terminology:
 * - UP: pressure data (16 to 19 bits)
 * - UT: temperature data (16  bits)
 *
 * @author Pier Colina
 * @date 20/06/2024
 * @version 1.0
 *
 */
#ifndef BMP180_H
#define BMP180_H

#pragma once

#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"

// BMP180 I2C Device Address
#define BMP180_DEV_ADD 0x77

// EEPROM Calibration Coefficients Addresses
#define BMP180_CC_NUM 11
#define BMP180_AC1_MSB 0xAA
#define BMP180_AC1_LSB 0xAB
#define BMP180_AC2_MSB 0xAC
#define BMP180_AC2_LSB 0xAD
#define BMP180_AC3_MSB 0xAE
#define BMP180_AC3_LSB 0xAF
#define BMP180_AC4_MSB 0xB0
#define BMP180_AC4_LSB 0xB1
#define BMP180_AC5_MSB 0xB2
#define BMP180_AC5_LSB 0xB3
#define BMP180_AC6_MSB 0xB4
#define BMP180_AC6_LSB 0xB5
#define BMP180_B1_MSB 0xB6
#define BMP180_B1_LSB 0xB7
#define BMP180_B2_MSB 0xB8
#define BMP180_B2_LSB 0xB9
#define BMP180_MB_MSB 0xBA
#define BMP180_MB_LSB 0xBB
#define BMP180_MC_MSB 0xBC
#define BMP180_MC_LSB 0xBD
#define BMP180_MD_MSB 0xBE
#define BMP180_MD_LSB 0xBF

// Data Registers
#define BMP180_ADC_OUT_XLSB 0xF8
#define BMP180_ADC_OUT_LSB 0xF7
#define BMP180_ADC_OUT_MSB 0xF6

// Measurement Control Register (MCR)
#define BMP180_CTRL_REG 0xF4

// Trigger UT reading command (MCR)
#define BMP180_TEMP_CMD 0X2E

// Trigger UP reading command (MCR) (The value for pressure control command must be modified with OSS defined by the user)
#define BMP180_PRESS_CMD 0X34

// Soft Reset Register
#define BMP180_SR_REG 0xE0
#define BMP180_SR_CMD 0xB6

// Chip ID Register
#define BPM180_CHIP_ID 0xD0


/// @brief 
typedef struct
{
    int oss;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
    int32_t UT;
    int32_t UP;
    int32_t B5;
} BMP180_DATA;


/// @brief Initialize the library. Configures the I2C master bus and adds the BMP180 I2C device address to the bus.
///        Populates the BMP180_DATA stucture with the sensor's calibration coefficients used for Pressure and Temperature calculation
///        and sets the sensor oversampling settings.
/// @param oss Desired oversampling setting. The mode (ultra low power, standard, high, ultra high resolution) can be selected by the variable
///            oversampling_setting (0, 1, 2, 3)
/// @return ESP_OK on success. ESP_ERR on failure.
esp_err_t BMP180_init(int oss);

/// @brief Deinitialize the library. Removes the I2C device and recycles the I2C master bus.
/// @return ESP_OK on success. ESP_ERR on failure.
esp_err_t BMP180_deinit();

/// @brief Performs a temperature measurement.
/// @param temperature Pointer to the variabe that'll hold the temperature data.
void BMP180_get_temp(float *temperature);

/// @brief Performs a pressure measurement.
/// @param pressure Pointer to the variabe that'll hold the pressure data.
void BMP180_get_press(float *pressure);

/// @brief Prints the sensor's calibration coefficients for debugging purposes.
void print_cc();

#endif