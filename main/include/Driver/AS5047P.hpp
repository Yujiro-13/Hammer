#ifndef AS5047P_HPP
#define AS5047P_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <cstring>
#include "sensor.hpp"

#define AS5047P_WHO_AM_I 0x70
#define AS5047P_READ_FLAG 0x4000
#define ENC_MAX 16384
#define ENC_HALF 8192

class AS5047P : public Sensor
{
public:
    AS5047P(spi_host_device_t bus,gpio_num_t cs);
    ~AS5047P();

    void GetData(t_sens_data *_sens) override;
    uint16_t readAngle();
    void ShowAngle();

private:
    uint8_t CalcParity(uint16_t data);
    uint16_t read16(uint16_t reg);
    esp_err_t err;
    spi_transaction_t cmd;
    spi_device_handle_t _spi;
    spi_bus_config_t bus_enc;
    spi_device_interface_config_t dev_enc;
    gpio_num_t _cs;

    t_sens_data *sens;
    
};

#endif