/**
 * @file esp_bmp180.h
 * @author JanG175
 * @brief ESP-IDF component for the BMP180 temperature and pressure sensor (I2C only)
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// #define BMP180_I2C_INIT      1 // uncomment to initialize I2C driver

#define BMP180_I2C_ADDRESS   0x77 // 0b01110111

#define BMP180_MAX_FREQ      3400000 // 3.4 MHz

#define BMP180_TIMEOUT_MS    100

// register map
#define BMP180_OUT_XLSB      0xF8
#define BMP180_OUT_LSB       0xF7
#define BMP180_OUT_MSB       0xF6
#define BMP180_CTRL_MEAS     0xF4
#define BMP180_RESET         0xE0
#define BMP180_ID            0xD0
#define BMP180_CALIB_0       0xAA
#define BMP180_CALIB_21      0xBF

// reset states
#define BMP180_RESET_0       0x00
#define BMP180_RESET_1       0x80
#define BMP180_RESET_2       0x55

// physical constants
#define P0   101325.0 // Pa

typedef struct
{
    uint8_t i2c_addr;
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
} bmp180_conf_t;

// BMP180 resolutions
enum bmp180_res
{
    BMP180_ULTRA_LOW_POWER = 0,
    BMP180_STANDARD,
    BMP180_HIGH_RES,
    BMP180_ULTRA_HIGH_RES
};


void bmp180_init(bmp180_conf_t bmp, enum bmp180_res res);

void bmp180_deinit(bmp180_conf_t bmp);

void bmp180_read_id(bmp180_conf_t bmp, uint8_t* id);

void bmp180_reset(bmp180_conf_t bmp);

void bmp180_read_ctrl_meas(bmp180_conf_t bmp, uint8_t* ctrl_meas);

void bmp180_write_ctrl_meas(bmp180_conf_t bmp, uint8_t* ctrl_meas);

void bmp180_read_temp_and_press(bmp180_conf_t bmp, enum bmp180_res res, float* temp_C, float* press_Pa);

void bmp180_read_height(bmp180_conf_t bmp, enum bmp180_res res, float* height_m);
