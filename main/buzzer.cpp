#include "include/Driver/Buzzer.hpp"

BUZZER::BUZZER(ledc_channel_t channel, ledc_timer_t timer, gpio_num_t pin){
    _channel = channel;
    _timer = timer;

    ledc_timer_config_t ledc_timer = {};

    //ledc_timer.speed_mode       = LEDC_MODE;
    //ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.timer_num        = _timer;
    ledc_timer.freq_hz          = 4000;  
    //ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    
    //ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {};

    //ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = _channel;
    ledc_channel.timer_sel      = _timer;
    //ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = pin;
    ledc_channel.duty           = 512; // Set duty to 0%
    ledc_channel.hpoint         = 0;

    //ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
BUZZER::~BUZZER(){}

void BUZZER::freq(uint32_t freq){
    /*ledc_set_duty(LEDC_MODE,_channel,512);
    ledc_update_duty(LEDC_MODE,_channel);
    ledc_set_freq(LEDC_MODE,_timer,freq);*/
}

void BUZZER::stop(){
    /*ledc_set_duty(LEDC_MODE,_channel,0);
    ledc_update_duty(LEDC_MODE,_channel);*/
}


/*void BUZZER::music(enum melody m){
    freq(m);
    //vTaskDelay(100/portTICK_PERIOD_MS);
    stop();
    //vTaskDelay(50/portTICK_PERIOD_MS);
}*/

void BUZZER::play(){
    
    while(1){
        /*
        //birhday song
        music(C);
        music(C);
        music(D);
        music(C);
        music(F);
        music(E);
        music(C);
        music(C);
        music(D);
        music(C);
        music(G);
        music(F);
        music(C);
        music(C);
        music(C);
        music(A);
        music(F);
        music(E);
        music(D);
        music(B);
        music(B);
        music(A);
        music(F);
        music(G);
        music(F);
        stop();
        */
        //vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }

