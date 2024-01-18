#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "../structs.hpp"
#include "peripheral.hpp"


class Motor
{
public:
    Motor(gpio_num_t ph_pin_R, gpio_num_t en_pin_R, gpio_num_t ph_pin_L, gpio_num_t en_pin_L, gpio_num_t fan_pin);
    ~Motor();
    void setMotorSpeed(float spdR, float spdL, float fan);
    void sincurve();
private:
    
    float spdR,spdL;
    float fan;
    float t = 0;
};

#endif