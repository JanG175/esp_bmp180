# ESP-IDF component for BMP180 I2C pressure sensor (I2C only)

## Notes
* If `i2c_master_bus_handle_t bus_handle` is defined outside of this component, please comment out `#define BMP180_I2C_INIT      1 // uncomment to initialize I2C driver` in `include/esp_bmp180.h`.
* Pressure and temperature readings are blocking.

## Sources
* https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf