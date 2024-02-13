#ifndef BUZZER_HPP
#define BUZZER_HPP

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include <iostream>
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

class BUZZER
{
public:

        BUZZER(gpio_num_t);
        ~BUZZER();
        void play(uint32_t = 1000, uint32_t = 1000);

    private:
        const char *TAG = "BUZZER";
        const uint32_t RMT_RESOLUTION = 1000000;
        typedef struct{
            uint32_t freq_hz;
            uint32_t duration_ms;
        } buzzer_score_t;

        typedef struct{
            rmt_encoder_t base;
            rmt_encoder_t *copy_encoder;
            uint32_t resolution;
        } buzzer_score_encoder_t;


        rmt_channel_handle_t buzzer_ch = NULL;
        rmt_encoder_handle_t buzzer_enc = NULL;
        rmt_copy_encoder_config_t copy_enc = {};

        esp_err_t rmt_encoder_create();
        static size_t rmt_encoder(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
        static esp_err_t rmt_encoder_delete(rmt_encoder_t *encoder);
        static esp_err_t rmt_encoder_reset(rmt_encoder_t *encoder);
};

#endif
