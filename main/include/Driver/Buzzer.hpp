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
        typedef struct melody
        {
            uint32_t A = 538;
            uint32_t B = 604;
            uint32_t C = 678;
            uint32_t D = 718;
            uint32_t E = 806;
            uint32_t F = 905;
            uint32_t G = 1016;
        } melody;

        BUZZER(ledc_channel_t channel, ledc_timer_t timer, gpio_num_t pin);
        ~BUZZER();

        void freq(uint32_t freq);
        void volume(uint32_t duty);
        void stop();
        void play();
        void music(uint32_t f);

    private:
        ledc_channel_t _channel;
        ledc_timer_t _timer;
        gpio_num_t BUZZER_PIN = GPIO_NUM_14;
        ledc_channel_t BUZZER_CH = LEDC_CHANNEL_0;
        ledc_timer_t BUZZER_TIMER = LEDC_TIMER_0;
};

#endif
