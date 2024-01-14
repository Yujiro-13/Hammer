#ifndef BUZZER_HPP
#define BUZZER_HPP

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <iostream>
#include "../structs.hpp"

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits

class BUZZER
{
public:
    BUZZER(ledc_channel_t channel, ledc_timer_t timer, gpio_num_t pin);
    ~BUZZER();

    void freq(uint32_t freq);
    void volume(uint32_t duty);
    void stop();
    //void play();
    //void music(enum melody m);

private:
    ledc_channel_t _channel;
    ledc_timer_t _timer;
    gpio_num_t BUZZER_PIN = GPIO_NUM_14;
    ledc_channel_t BUZZER_CH = LEDC_CHANNEL_0;
    ledc_timer_t BUZZER_TIMER = LEDC_TIMER_0;
    enum melody
    {
        A = 538,
        B = 604,
        C = 678,
        D = 718,
        E = 806,
        F = 905,
        G = 1016,
    };
};

#endif
