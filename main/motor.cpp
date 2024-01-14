#include "include/Driver/Motor.hpp"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             250000    // 250KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks


//bdc_motor_handle_t motor_r = NULL;
//bdc_motor_handle_t motor_l = NULL;

//mcpwm_cmpr_handle_t comparator_r = NULL;
//mcpwm_cmpr_handle_t comparator_l = NULL;


Motor::Motor(gpio_num_t ph_pin_R, gpio_num_t en_pin_R, gpio_num_t ph_pin_L, gpio_num_t en_pin_L, gpio_num_t fan_pin){
    //モタドラ
    //MODE GPIO40
    /*gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 
        (1ULL<<GPIO_NUM_40) | (1ULL<<BDC_L_MCPWM_GPIO_PH) | (1ULL<<BDC_R_MCPWM_GPIO_PH) | (1ULL<<BDC_L_MCPWM_GPIO_EN) | (1ULL<<BDC_R_MCPWM_GPIO_EN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(GPIO_NUM_40, 1);
    gpio_set_level(BDC_L_MCPWM_GPIO_PH, 0);
    gpio_set_level(BDC_R_MCPWM_GPIO_PH, 0);
    gpio_set_level(BDC_L_MCPWM_GPIO_EN, 0);
    gpio_set_level(BDC_R_MCPWM_GPIO_EN, 0);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config;
    timer_config.group_id = 0,
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;
    timer_config.period_ticks = BDC_MCPWM_DUTY_TICK_MAX;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0; // operator must be in the same group to the timer
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

     //コンパレーターを作る
    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_r));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_l));

    //ジェネレーターを作る
    mcpwm_gen_handle_t generator_r = NULL;
    mcpwm_gen_handle_t generator_l = NULL;
    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = BDC_R_MCPWM_GPIO_EN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator_r));
    generator_config.gen_gpio_num = BDC_L_MCPWM_GPIO_EN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator_l));

    // set the initial compare value
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_r, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_l, 0));

    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_r,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_l,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_r,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_r, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_l,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_l, MCPWM_GEN_ACTION_LOW)));
    
    //タイマーを有効にしてスタート
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    //吸引ファン
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = 100*1000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = FAN_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_1,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));*/

    setMotorSpeed(0,0,0);
}
Motor::~Motor(){}

void Motor::setMotorSpeed(float spdR, float spdL,float fan){

    /*if(spdR > 0)    gpio_set_level(BDC_R_MCPWM_GPIO_PH, 1); //  右モータ反転させてつけた
    else            gpio_set_level(BDC_R_MCPWM_GPIO_PH, 0);
    if(spdL > 0)    gpio_set_level(BDC_L_MCPWM_GPIO_PH, 1);
    else            gpio_set_level(BDC_L_MCPWM_GPIO_PH, 0);

    float dutyR = fabs(spdR);
    float dutyL = fabs(spdL);

    if(dutyR > 1.0) dutyR = 1.0;
    if(dutyL > 1.0) dutyL = 1.0;

    esp_err_t MCPWM_ERROR_R = mcpwm_comparator_set_compare_value(comparator_r, (dutyR) * (BDC_MCPWM_DUTY_TICK_MAX - 1));
    esp_err_t MCPWM_ERROR_L = mcpwm_comparator_set_compare_value(comparator_l, (dutyL) * (BDC_MCPWM_DUTY_TICK_MAX - 1));

    if(MCPWM_ERROR_R != ESP_OK) printf("MCPWM_ERROR_R : %f    spdR : %f\n", dutyR, spdR);


    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, fan * 256);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);*/
}

void Motor::sincurve(){
    float spdR,spdL;
    float fan;
    float t = 0;
    while(1){
        spdR = sin(t);
        spdL = sin(t);
        fan = 0.5 * sin(t) + 0.5;
        if(isnan(spdR)){
            printf("%f,\r\n",t);
            
        }
        setMotorSpeed(0.4,0.0,fan);
        t += 0.01;
        //vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}