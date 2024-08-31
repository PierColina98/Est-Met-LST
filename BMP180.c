#include "BMP180.h"

void BMP180_get_cal_coeff();
int BMP180_check_conn();
int BMP180_check_conv();
void BMP180_read_temp();
void BMP180_read_press();
void BMP180_compute_B5();
void BMP180_soft_reset();

BMP180_DATA bmpdata = {0};

esp_err_t BMP180_init(int oss)
{
    bmpdata.oss = oss;
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
        .device_address = 0x77,
        .scl_speed_hz = 100000,
    };

    if (i2c_new_master_bus(&bus_cfg, &bmpdata.bus_handle) != ESP_OK)
    {
        ESP_LOGI("I2C", "MASTER BUS ERROR.");
        return ESP_ERR_NOT_FINISHED;
    }

    if (i2c_master_probe(bmpdata.bus_handle, 0x77, -1) != ESP_OK)
    {
        ESP_LOGI("I2C", "DEV ADDR NOT FOUND.");
        i2c_del_master_bus(bmpdata.bus_handle);
        return ESP_ERR_NOT_FINISHED;
    }
    else
    {
        if (i2c_master_bus_add_device(bmpdata.bus_handle, &dev_cfg, &bmpdata.dev_handle) != ESP_OK)
        {
            ESP_LOGI("I2C", "DEV HANDLER ERROR.");
            i2c_del_master_bus(bmpdata.bus_handle);
            return ESP_ERR_NOT_FINISHED;
        }
    }
    BMP180_get_cal_coeff();
    ESP_LOGI("BMP180", "INITIALIZED!");
    return ESP_OK;
};

esp_err_t BMP180_deinit()
{
    if(i2c_master_bus_rm_device(bmpdata.dev_handle) != ESP_OK){
        ESP_LOGI("I2C", "DEV HANDLE REMOVE ERROR.");
        return ESP_ERR_NOT_FINISHED;
    }else{
        if (i2c_del_master_bus(bmpdata.bus_handle) != ESP_OK){
            ESP_LOGI("I2C", "BUS HANDLE REMOVE ERROR.");
            return ESP_ERR_NOT_FINISHED;
        }else{
            return ESP_OK;
        }
    }
};

void BMP180_get_cal_coeff()
{
    uint8_t cc_register[2];
    uint8_t read_buffer[2];
    for (int i = 0; i < BMP180_CC_NUM; i++)
    {
        switch (i)
        {
        case 0:
        {
            cc_register[0] = BMP180_AC1_MSB;
            cc_register[1] = BMP180_AC1_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.AC1 = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 1:
        {
            cc_register[0] = BMP180_AC2_MSB;
            cc_register[1] = BMP180_AC2_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.AC2 = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 2:
        {
            cc_register[0] = BMP180_AC3_MSB;
            cc_register[1] = BMP180_AC3_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.AC3 = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 3:
        {
            cc_register[0] = BMP180_AC4_MSB;
            cc_register[1] = BMP180_AC4_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.AC4 = (uint16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 4:
        {
            cc_register[0] = BMP180_AC5_MSB;
            cc_register[1] = BMP180_AC5_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.AC5 = (uint16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 5:
        {
            cc_register[0] = BMP180_AC6_MSB;
            cc_register[1] = BMP180_AC6_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.AC6 = (uint16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 6:
        {
            cc_register[0] = BMP180_B1_MSB;
            cc_register[1] = BMP180_B1_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.B1 = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 7:
        {
            cc_register[0] = BMP180_B2_MSB;
            cc_register[1] = BMP180_B2_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.B2 = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 8:
        {
            cc_register[0] = BMP180_MB_MSB;
            cc_register[1] = BMP180_MB_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.MB = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 9:
        {
            cc_register[0] = BMP180_MC_MSB;
            cc_register[1] = BMP180_MC_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.MC = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        case 10:
        {
            cc_register[0] = BMP180_MD_MSB;
            cc_register[1] = BMP180_MD_LSB;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &cc_register[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
            bmpdata.MD = (int16_t)((read_buffer[0] << 8) | read_buffer[1]);
            break;
        }
        default:
            break;
        }
    }
};

int BMP180_check_conn()
{
    uint8_t chip_id_reg = BPM180_CHIP_ID;
    uint8_t read_buffer;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &chip_id_reg, sizeof(uint8_t), &read_buffer, sizeof(uint8_t), -1));
    if (read_buffer != 0x55)
    {
        return -1;
    }
    else
    {
        return 0;
    }
};

int BMP180_check_conv()
{
    uint8_t ctrl_reg = BMP180_CTRL_REG;
    uint8_t read_buffer;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &ctrl_reg, sizeof(uint8_t), &read_buffer, sizeof(uint8_t), -1));
    if (read_buffer & (0x01 << 5))
    {
        return -1;
    }
    else
    {
        return 0;
    }
};

void BMP180_read_temp()
{
    uint8_t temp_ctrl_reg[2] = {BMP180_CTRL_REG, BMP180_TEMP_CMD};
    uint8_t adc_data_reg[2] = {BMP180_ADC_OUT_MSB, BMP180_ADC_OUT_LSB};
    uint8_t read_buffer[2];
    i2c_master_transmit_multi_buffer_info_t cmd[2] = {
        {.write_buffer = &temp_ctrl_reg[0], .buffer_size = 1},
        {.write_buffer = &temp_ctrl_reg[1], .buffer_size = 1}};
    ESP_ERROR_CHECK(i2c_master_multi_buffer_transmit(bmpdata.dev_handle, cmd, sizeof(cmd) / sizeof(i2c_master_transmit_multi_buffer_info_t), -1));
    vTaskDelay(5 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &adc_data_reg[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &adc_data_reg[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
    bmpdata.UT = (read_buffer[0] << 8) | read_buffer[1];
};

void BMP180_read_press()
{
    uint8_t press_ctrl_cmd[2] = {BMP180_CTRL_REG, BMP180_PRESS_CMD + (bmpdata.oss << 6)};
    uint8_t adc_data_reg[3] = {BMP180_ADC_OUT_MSB, BMP180_ADC_OUT_LSB, BMP180_ADC_OUT_XLSB};
    uint8_t read_buffer[3];
    i2c_master_transmit_multi_buffer_info_t cmd[2] = {
        {.write_buffer = &press_ctrl_cmd[0], .buffer_size = 1},
        {.write_buffer = &press_ctrl_cmd[1], .buffer_size = 1}};
    ESP_ERROR_CHECK(i2c_master_multi_buffer_transmit(bmpdata.dev_handle, cmd, sizeof(cmd) / sizeof(i2c_master_transmit_multi_buffer_info_t), -1));
    vTaskDelay(2 + (3 << bmpdata.oss) / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &adc_data_reg[0], sizeof(uint8_t), &read_buffer[0], sizeof(uint8_t), -1));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &adc_data_reg[1], sizeof(uint8_t), &read_buffer[1], sizeof(uint8_t), -1));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bmpdata.dev_handle, &adc_data_reg[2], sizeof(uint8_t), &read_buffer[2], sizeof(uint8_t), -1));
    bmpdata.UP = ((read_buffer[0] << 16) | (read_buffer[1] << 8) | read_buffer[2]) >> (8 - bmpdata.oss);
};

void BMP180_compute_B5(){
    int32_t X1 = (bmpdata.UT - (int32_t)bmpdata.AC6) * ((int32_t)bmpdata.AC5) >> 15;
    int32_t X2 = ((int32_t)bmpdata.MC << 11) / (X1 + (int32_t)bmpdata.MD);
    bmpdata.B5 = X1 + X2;
}

void BMP180_get_temp(float *temperature)
{
    BMP180_read_temp();
    BMP180_read_press();
    BMP180_compute_B5();
    int32_t T = (bmpdata.B5 + 8) >> 4;
    *temperature = T * 0.1;
};

void BMP180_get_press(float *pressure)
{
    BMP180_read_temp();
    BMP180_read_press();
    BMP180_compute_B5();
    int32_t P = 0;
    int32_t B6 = bmpdata.B5 - 4000;
    int32_t X1 = ((int32_t)bmpdata.B2 * ((B6 * B6) >> 12)) >> 11;
    int32_t X2 = ((int32_t)bmpdata.AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)bmpdata.AC1 * 4 + X3) << bmpdata.oss) + 2) / 4;
    X1 = ((int32_t)bmpdata.AC3 * B6) >> 13;
    X2 = ((int32_t)bmpdata.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2)+ 2) >> 2;
    uint32_t B4 = ((uint32_t)bmpdata.AC4 * (uint32_t)(X3 + 32768) ) >> 15;
    uint32_t B7 = ((uint32_t)bmpdata.UP - B3) * (uint32_t)(50000 >> bmpdata.oss);
    if (B7 < 0x80000000)
    {
        P = (B7 * 2) / B4;
    }
    else
    {
        P = (B7 / B4) * 2;
    }
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + (int32_t)3791) >> 4);
    *pressure = P * 0.01;
};

void BMP180_soft_reset()
{
    uint8_t reset_reg[2] = {BMP180_SR_REG, BMP180_SR_CMD};
    ESP_ERROR_CHECK(i2c_master_transmit(bmpdata.dev_handle, reset_reg, 2, -1));
};

void print_cc()
{
    ESP_LOGI("BMP180", "Calibration coefficients: ");
    printf("AC1: %d\n", bmpdata.AC1);
    printf("AC2: %d\n", bmpdata.AC2);
    printf("AC3: %d\n", bmpdata.AC3);
    printf("AC4: %u\n", bmpdata.AC4);
    printf("AC5: %u\n", bmpdata.AC5);
    printf("AC6: %u\n", bmpdata.AC6);
    printf("B1: %d\n", bmpdata.B1);
    printf("B2: %d\n", bmpdata.B2);
    printf("MB: %d\n", bmpdata.MB);
    printf("MC: %d\n", bmpdata.MC);
    printf("MD: %d\n", bmpdata.MD);
    printf("B5: %ld\n", bmpdata.B5);
    printf("UT: %ld\n", bmpdata.UT);
    printf("UP: %ld\n", bmpdata.UP);
}