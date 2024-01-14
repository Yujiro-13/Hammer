#include "include/Driver/adc.hpp"
//#include <rom/ets_sys.h>

//adc_oneshot_unit_handle_t adc1;

// ADCのピン、チャンネルは構造体でそれぞれまとめたので、一括でクラス内で宣言したインスタンスに渡す
ADC::ADC(IRLED_FR &led_fr, IRLED_FL &led_fl, IRLED_R &led_r, IRLED_L &led_l, adc_channel_t VBATT) : led_FR(led_fr), led_FL(led_fl), led_R(led_r), led_L(led_l)
{
    /*adc_oneshot_unit_init_cfg_t adc1_init = {};
    adc1_init.unit_id = ADC_UNIT_1;

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_init, &adc1));

    adc_oneshot_chan_cfg_t adc1_chan = {};
    adc1_chan.bitwidth = ADC_BITWIDTH_DEFAULT;
    adc1_chan.atten = ADC_ATTEN_DB_6;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, VBATT_CHANNEL, &adc1_chan)); // バッテリー電圧
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, led_FR.channel, &adc1_chan)); // FR
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, led_R.channel, &adc1_chan)); // R
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, led_L.channel, &adc1_chan)); // L
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, led_FL.channel, &adc1_chan)); // FL

    gpio_config_t ir_conf = {};
    ir_conf.mode = GPIO_MODE_OUTPUT;
    ir_conf.pin_bit_mask = (1ULL << led_FR.pin) | (1ULL << led_FL.pin) | (1ULL << led_R.pin) | (1ULL << led_L.pin);
    ESP_ERROR_CHECK(gpio_config(&ir_conf));*/
    SetIRLED(0b0000);
}

ADC::~ADC()
{
    //adc_oneshot_delete(adc1);
}

float ADC::BatteryVoltage()
{
    int _raw = 0;
    //ESP_ERROR_CHECK(adc_oneshot_read(adc1, VBATT_CHANNEL, &_raw));
    return (float)((float)(_raw) * (2.2 / 1000.0));
}

void ADC::ReadSensor(int *sensors, uint8_t mask)
{
    /*if (mask & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_FL.channel, &sensors[0]));
    if ((mask >> 1) & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_L.channel, &sensors[1]));
    if ((mask >> 2) & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_R.channel, &sensors[2]));
    if ((mask >> 3) & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_FR.channel, &sensors[3]));
        */
}

void ADC::SetIRLED(uint8_t led)
{
    /*gpio_set_level(led_FR.pin, led & 1);
    gpio_set_level(led_R.pin, (led >> 1) & 1);
    gpio_set_level(led_L.pin, (led >> 2) & 1);
    gpio_set_level(led_FL.pin, (led >> 3) & 1);
    */
}

void ADC::WallSensor()
{

    ReadSensor(before, 0b1111); // 全消灯での値を取得

    SetIRLED(0b1001);            // fl,fr点灯
    //ets_delay_us(300);           // 100us待つ
    ReadSensor(sensors, 0b1001); // fl,fr点灯での値を取得

    SetIRLED(0b0110);                   // l,r点灯
    //ets_delay_us(300);                  // 100us待つ
    ReadSensor(sensors, 0b0110);        // l,r点灯での値を取得
    SetIRLED(0b0000);                   // 全消灯
    //vTaskDelay(1 / portTICK_PERIOD_MS); // 1ms待つ

    sens->wall.val.fl = sensors[0] - before[0];
    sens->wall.val.l = sensors[1] - before[1];
    sens->wall.val.r = sensors[2] - before[2];
    sens->wall.val.fr = sensors[3] - before[3];
}

void ADC::GetData(t_sens_data *_sens) // オーバーライドしているから引数をvoid *pvparamにできない
{
    sens = _sens;
}