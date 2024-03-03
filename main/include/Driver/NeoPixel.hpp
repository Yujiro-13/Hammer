#ifndef NEOPIXEL_HPP
#define NEOPIXEL_HPP

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include <iostream>
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"

class NeoPixel{
    public:
        typedef struct{
            uint32_t h;
            uint32_t s;
            uint32_t v;
        } hsv_t;
        NeoPixel(gpio_num_t);
        ~NeoPixel();
        void set_hsv(hsv_t);

    private:
        const char *TAG = "NeoPixel";
        const uint32_t RMT_RESOLUTION = 10000000;

        typedef struct{
            rmt_encoder_t base;
            rmt_encoder_t *bytes_encoder;
            rmt_encoder_t *copy_encoder;
            int state;
            rmt_symbol_word_t reset_code;
        } neopixel_encoder_t;

        rmt_channel_handle_t neopixel_ch = NULL;
        rmt_encoder_handle_t neopixel_enc = NULL;
        rmt_copy_encoder_config_t copy_enc = {};
        rmt_transmit_config_t tx_config = {};

        esp_err_t encoder_rmt_create();
        static size_t encoder_rmt(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
        static esp_err_t encoder_delete(rmt_encoder_t *encoder);
        static esp_err_t encoder_reset(rmt_encoder_t *encoder);

        uint8_t grb[4] = {0,0,0,0};

        void hsv2grb(hsv_t);
};

#endif