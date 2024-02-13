#include "include/Driver/Buzzer.hpp"

BUZZER::BUZZER(gpio_num_t pin){

    rmt_tx_channel_config_t _tx_config;
    memset(&_tx_config, 0, sizeof(_tx_config));
    _tx_config.gpio_num = pin;
    _tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    _tx_config.mem_block_symbols = 64;
    _tx_config.resolution_hz = RMT_RESOLUTION;
    _tx_config.trans_queue_depth = 10;

    ESP_ERROR_CHECK(rmt_new_tx_channel(&_tx_config, &buzzer_ch));
    ESP_ERROR_CHECK(rmt_encoder_create());
    ESP_ERROR_CHECK(rmt_enable(buzzer_ch));
}

BUZZER::~BUZZER(){}

esp_err_t BUZZER::rmt_encoder_create(){
    esp_err_t ret = ESP_OK;

    buzzer_score_encoder_t *score = NULL;
    score = (buzzer_score_encoder_t*)calloc(1, sizeof(buzzer_score_encoder_t));
    ESP_GOTO_ON_FALSE(score, ESP_ERR_NO_MEM, err, this->TAG, "calloc failed");
    score->base.encode = this->rmt_encoder;
    score->base.del = this->rmt_encoder_delete;
    score->base.reset = this->rmt_encoder_reset;
    score->resolution = RMT_RESOLUTION;
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&this->copy_enc,&score->copy_encoder), err, TAG, "rmt_new_copy_encoder failed");
    buzzer_enc = &score->base;
    return ESP_OK;

err:
    if(score){
        if(score->copy_encoder){
            rmt_del_encoder(score->copy_encoder);
        }
        free(score);
    }
    return ret;
}

size_t BUZZER::rmt_encoder(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state){
    buzzer_score_encoder_t *score_enc = __containerof(encoder, buzzer_score_encoder_t, base);
    rmt_encoder_handle_t copy_encoder = score_enc->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    buzzer_score_t *score = (buzzer_score_t *)primary_data;
    uint32_t rmt_raw_symbol_duration = score_enc->resolution / score->freq_hz / 2;
    rmt_symbol_word_t musical_score_rmt_symbol;
    musical_score_rmt_symbol.level0 = 0;
    musical_score_rmt_symbol.level1 = 1;
    musical_score_rmt_symbol.duration0 = rmt_raw_symbol_duration;
    musical_score_rmt_symbol.duration1 = rmt_raw_symbol_duration;

    size_t encoded_symbols = copy_encoder->encode(copy_encoder, channel, &musical_score_rmt_symbol, sizeof(musical_score_rmt_symbol), &session_state);
    *ret_state = session_state;
    return encoded_symbols;
}

esp_err_t BUZZER::rmt_encoder_delete(rmt_encoder_t *encoder){
    buzzer_score_encoder_t *score_enc = __containerof(encoder, buzzer_score_encoder_t, base);
    if(score_enc->copy_encoder){
        rmt_del_encoder(score_enc->copy_encoder);
    }
    free(score_enc);
    return ESP_OK;
}

esp_err_t BUZZER::rmt_encoder_reset(rmt_encoder_t *encoder){
    buzzer_score_encoder_t *score_enc = __containerof(encoder, buzzer_score_encoder_t, base);
    if(score_enc->copy_encoder){
        rmt_encoder_reset(score_enc->copy_encoder);
    }
    return ESP_OK;
}

void BUZZER::play(uint32_t freq_hz, uint32_t duration){
    buzzer_score_t score = {
        .freq_hz = freq_hz,
        .duration_ms = duration
    };
    this->play_melody(&score,1);
}

void BUZZER::play_melody(buzzer_score_t* score,int len){
    rmt_tx_wait_all_done(this->buzzer_ch, portMAX_DELAY);
    for(int i=0;i<len;i++){
        rmt_transmit_config_t _tx_config;
        _tx_config.loop_count = score[i].duration_ms * score[i].freq_hz / 1000;
        ESP_ERROR_CHECK(rmt_transmit(this->buzzer_ch, this->buzzer_enc,&score[i], sizeof(buzzer_score_t),&_tx_config));
    }
}