#ifndef PERIPHERAL_HPP
#define PERIPHERAL_HPP

/*各モジュールに使用しているピン、チャンネルを書いておく*/

#include <iostream>
#include <string>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "sdkconfig.h"
#include "../structs.hpp"

constexpr gpio_num_t LED_SDA = GPIO_NUM_37;
constexpr gpio_num_t LED_SCL = GPIO_NUM_38;
constexpr int LED_FREQ = 1000 * 1000;
constexpr uint8_t LED_ADRS = 0x62;

constexpr gpio_num_t IMU_MISO = GPIO_NUM_11;
constexpr gpio_num_t IMU_MOSI = GPIO_NUM_12;
constexpr gpio_num_t IMU_CLK = GPIO_NUM_2;
constexpr gpio_num_t IMU_CS = GPIO_NUM_7;

constexpr gpio_num_t BUZZER_PIN = GPIO_NUM_14;
constexpr ledc_channel_t BUZZER_CH = LEDC_CHANNEL_0;
constexpr ledc_timer_t BUZZER_TIMER = LEDC_TIMER_0;

constexpr gpio_num_t ENC_MISO = GPIO_NUM_35;
constexpr gpio_num_t ENC_MOSI = GPIO_NUM_36;
constexpr gpio_num_t ENC_CLK = GPIO_NUM_34;
constexpr gpio_num_t ENC_CS_R = GPIO_NUM_1;
constexpr gpio_num_t ENC_CS_L = GPIO_NUM_6;

/*constexpr gpio_num_t IRLED_FR = GPIO_NUM_33;
constexpr gpio_num_t IRLED_FL = GPIO_NUM_17;
constexpr gpio_num_t IRLED_R = GPIO_NUM_21;
constexpr gpio_num_t IRLED_L = GPIO_NUM_18;*/

constexpr adc_channel_t VBATT_CHANNEL = ADC_CHANNEL_2;

typedef struct
{
    gpio_num_t    pin     = GPIO_NUM_33;
    adc_channel_t channel = ADC_CHANNEL_3;
} IRLED_FR;

typedef struct
{
    gpio_num_t    pin     = GPIO_NUM_17;
    adc_channel_t channel = ADC_CHANNEL_9;
} IRLED_FL;

typedef struct
{
    gpio_num_t    pin     = GPIO_NUM_21;
    adc_channel_t channel = ADC_CHANNEL_4;
} IRLED_R;

typedef struct
{
    gpio_num_t    pin     = GPIO_NUM_18;
    adc_channel_t channel = ADC_CHANNEL_7;
} IRLED_L;

constexpr gpio_num_t BDC_R_MCPWM_GPIO_PH = GPIO_NUM_45;
constexpr gpio_num_t BDC_R_MCPWM_GPIO_EN = GPIO_NUM_46;
constexpr gpio_num_t BDC_L_MCPWM_GPIO_PH = GPIO_NUM_41;
constexpr gpio_num_t BDC_L_MCPWM_GPIO_EN = GPIO_NUM_42;
constexpr gpio_num_t FAN_PIN = GPIO_NUM_13;

/*IRLED_FR* LED_FR;
IRLED_FL* LED_FL;
IRLED_R* LED_R;
IRLED_L* LED_L;*/


#endif