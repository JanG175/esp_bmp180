/**
 * @file esp_bmp180.c
 * @author JanG175
 * @brief ESP-IDF component for the BMP180 temperature and pressure sensor (I2C only)
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_bmp180.h"

static float pressure_sea_level = P0;
#ifdef BMP180_I2C_INIT
static i2c_master_bus_handle_t bus_handle;
#else
extern i2c_master_bus_handle_t bus_handle;
#endif
static i2c_master_dev_handle_t dev_handle;

static const char *TAG = "BMP180";


/**
 * @brief compensate temperature and pressure readings using integer values
 * 
 * @param bmp struct with BMP180 parameters
 * @param res sensor resolution
 * @param UT temperature reading
 * @param T pointer to compensated temperature
 * @param UP pressure reading
 * @param P pointer to compensated pressure
*/
static void bmp280_compensate_T_P_int(bmp180_conf_t bmp, enum bmp180_res res, int32_t UT, float* T, int32_t UP, float* P)
{
    // read calibration data
    uint8_t dig_calib[22];
    uint8_t reg = BMP180_CALIB_0;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, dig_calib, 22, BMP180_TIMEOUT_MS));

    int16_t AC1 = (int16_t)dig_calib[0] << 8 | (int16_t)dig_calib[1];
    int16_t AC2 = (int16_t)dig_calib[2] << 8 | (int16_t)dig_calib[3];
    int16_t AC3 = (int16_t)dig_calib[4] << 8 | (int16_t)dig_calib[5];
    uint16_t AC4 = (uint16_t)dig_calib[6] << 8 | (uint16_t)dig_calib[7];
    uint16_t AC5 = (uint16_t)dig_calib[8] << 8 | (uint16_t)dig_calib[9];
    uint16_t AC6 = (uint16_t)dig_calib[10] << 8 | (uint16_t)dig_calib[11];
    int16_t B1 = (int16_t)dig_calib[12] << 8 | (int16_t)dig_calib[13];
    int16_t B2 = (int16_t)dig_calib[14] << 8 | (int16_t)dig_calib[15];
    int16_t MB = (int16_t)dig_calib[16] << 8 | (int16_t)dig_calib[17];
    int16_t MC = (int16_t)dig_calib[18] << 8 | (int16_t)dig_calib[19];
    int16_t MD = (int16_t)dig_calib[20] << 8 | (int16_t)dig_calib[21];

    // temperature
    int32_t X1 = ((UT - AC6) * AC5) >> 15;
    int32_t X2 = (MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;
    int32_t t = ((B5 + 8) >> 4);
    *T = (float)t / 10.0f;

    // pressure
    int32_t B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6) >> 12) >> 11;
    X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((AC1 * 4 + X3) << res) + 2) / 4;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> res);
    int32_t p = 0;
    if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
    else
        p = (B7 / B4) * 2;
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = (p + ((X1 + X2 + 3791) >> 4));
    *P = (float)p;
}


/**
 * @brief initialize the BMP180
 * 
 * @param bmp struct with BMP180 parameters
 * @param res sensor resolution
*/
void bmp180_init(bmp180_conf_t bmp, enum bmp180_res res)
{
#ifdef BMP180_I2C_INIT
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = bmp.i2c_port,
        .scl_io_num = bmp.scl_pin,
        .sda_io_num = bmp.sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
#endif

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmp.i2c_addr,
        .scl_speed_hz = bmp.i2c_freq
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // reset
    bmp180_reset(bmp);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // calibrate sea level pressure
    float press_Pa[10];
    float temp_C;

    for (uint32_t i = 0; i < 10; i++)
    {
        bmp180_read_temp_and_press(bmp, res, &temp_C, &press_Pa[i]);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    pressure_sea_level = 0.0f;
    for (uint32_t i = 0; i < 10; i++)
        pressure_sea_level += press_Pa[i];
    pressure_sea_level /= 10.0f;
}


/**
 * @brief deinitialize the BMP180
 * 
 * @param bmp struct with BMP180 parameters
*/
void bmp180_deinit(bmp180_conf_t bmp)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));

#ifdef BMP180_I2C_INIT
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
#endif
}


/**
 * @brief read BMP180 id register
 * 
 * @param bmp struct with BMP180 parameters
 * @param id pointer to id data
*/
void bmp180_read_id(bmp180_conf_t bmp, uint8_t* id)
{
    uint8_t reg = BMP180_ID;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, id, 1, BMP180_TIMEOUT_MS));
}


/**
 * @brief reset the BMP180
 * 
 * @param bmp struct with BMP180 parameters
*/
void bmp180_reset(bmp180_conf_t bmp)
{
    uint8_t data[2] = {BMP180_RESET_0, 0xB6};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, BMP180_TIMEOUT_MS));
}


/**
 * @brief read BMP180 ctrl_meas register
 * 
 * @param bmp struct with BMP180 parameters
 * @param ctrl_meas pointer to ctrl_meas data
*/
void bmp180_read_ctrl_meas(bmp180_conf_t bmp, uint8_t* ctrl_meas)
{
    uint8_t reg = BMP180_CTRL_MEAS;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, ctrl_meas, 1, BMP180_TIMEOUT_MS));

    // oss
    if ((*ctrl_meas & 0b11000000) == 0b00000000)
        ESP_LOGI(TAG, "oss: oversampling x1");
    else if ((*ctrl_meas & 0b11000000) == 0b01000000)
        ESP_LOGI(TAG, "oss: oversampling x2");
    else if ((*ctrl_meas & 0b11000000) == 0b10000000)
        ESP_LOGI(TAG, "oss: oversampling x4");
    else if ((*ctrl_meas & 0b11000000) == 0b11000000)
        ESP_LOGI(TAG, "oss: oversampling x8");

    // sco
    if ((*ctrl_meas & 0b00100000) == 0b00000000)
        ESP_LOGI(TAG, "sco: pending conversion");
    else if ((*ctrl_meas & 0b00100000) == 0b00100000)
        ESP_LOGI(TAG, "sco: conversion is complete");
}


/**
 * @brief write to BMP180 ctrl_meas register
 * 
 * @param bmp struct with BMP180 parameters
 * @param ctrl_meas pointer to ctrl_meas data
*/
void bmp180_write_ctrl_meas(bmp180_conf_t bmp, uint8_t* ctrl_meas)
{
    uint8_t data[2] = {BMP180_CTRL_MEAS, *ctrl_meas};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, BMP180_TIMEOUT_MS));
}


/**
 * @brief read temperature and pressure from BMP180 (this function blocks the task!)
 * 
 * @param bmp struct with BMP180 parameters
 * @param res sensor resolution
 * @param temp_C pointer to temperature in Celsius
 * @param press_Pa pointer to pressure in Pascals
*/
void bmp180_read_temp_and_press(bmp180_conf_t bmp, enum bmp180_res res, float* temp_C, float* press_Pa)
{
    uint8_t data[3] = {0, 0, 0};

    TickType_t last_time = xTaskGetTickCount();

    // read uncompensated temperature value
    uint8_t reg = 0x2E;
    bmp180_write_ctrl_meas(bmp, &reg);
    xTaskDelayUntil(&last_time, 5 / portTICK_PERIOD_MS);

    reg = BMP180_OUT_MSB;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 3, BMP180_TIMEOUT_MS));
    int32_t temp = (int32_t)((uint32_t)data[0] << 8) | ((uint32_t)data[1]);

    // read uncompensated pressure value
    last_time = xTaskGetTickCount();

    reg = 0x34;
    switch (res)
    {
        case BMP180_ULTRA_LOW_POWER:
            // oss = x1 (0b00)
            reg = reg & ~(0b00000011 << 6);
            bmp180_write_ctrl_meas(bmp, &reg);
            xTaskDelayUntil(&last_time, 5 / portTICK_PERIOD_MS);
            break;
        case BMP180_STANDARD:
            // oss = x2 (0b01)
            reg = reg & ~(1 << 7);
            reg = reg | (1 << 6);
            bmp180_write_ctrl_meas(bmp, &reg);
            xTaskDelayUntil(&last_time, 8 / portTICK_PERIOD_MS);
            break;
        case BMP180_HIGH_RES:
            // oss = x4 (0b10)
            reg = reg | (1 << 7);
            reg = reg & ~(1 << 6);
            bmp180_write_ctrl_meas(bmp, &reg);
            xTaskDelayUntil(&last_time, 14 / portTICK_PERIOD_MS);
            break;
        case BMP180_ULTRA_HIGH_RES:
            // oss = x8 (0b11)
            reg = reg | (0b00000011 << 6);
            bmp180_write_ctrl_meas(bmp, &reg);
            xTaskDelayUntil(&last_time, 26 / portTICK_PERIOD_MS);
            break;
        default:
            ESP_LOGE(TAG, "Invalid resolution!");
            return;
    }

    reg = BMP180_OUT_MSB;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 3, BMP180_TIMEOUT_MS));
    int32_t press = (int32_t)((((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2]) >> (8 - res));

    // compensate temperature and pressure
    bmp280_compensate_T_P_int(bmp, res, temp, temp_C, press, press_Pa);
}


/**
 * @brief read height from BMP180
 * 
 * @param bmp struct with BMP180 parameters
 * @param res sensor resolution
 * @param height_m pointer to height in meters
*/
void bmp180_read_height(bmp180_conf_t bmp, enum bmp180_res res, float* height_m)
{
    float press_Pa = 0.0f;
    float temp_C = 0.0f;

    bmp180_read_temp_and_press(bmp, res, &temp_C, &press_Pa);

    *height_m = 44330.0f * (1.0f - powf(press_Pa / pressure_sea_level, 1.0f / 5.255f));
}