#include <stdio.h>
#include <iostream>
#include <fstream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "MIcromouse/Base_task.hpp"
#include "include/MIcromouse/interrupt.hpp"


extern "C" void app_main(void)
{
    
    IRLED_FR LED_FR;
    IRLED_FL LED_FL;
    IRLED_R LED_R;
    IRLED_L LED_L;

    /* デバイスクラスのインスタンス生成と初期化 */   // 必ずmain関数内で行う

    Motor motor(BDC_R_MCPWM_GPIO_PH, BDC_R_MCPWM_GPIO_EN, BDC_L_MCPWM_GPIO_PH, BDC_L_MCPWM_GPIO_EN, FAN_PIN);
    PCA9632 led(I2C_NUM_0, LED_ADRS);
    BUZZER buzzer(BUZZER_CH, BUZZER_TIMER, BUZZER_PIN); 
    AS5047P enc_R(SPI3_HOST, ENC_CS_R,false);                 
    AS5047P enc_L(SPI3_HOST, ENC_CS_L,true);                
    MPU6500 imu(SPI2_HOST, IMU_CS);  
    ADC adc(LED_FR, LED_FL, LED_R, LED_L, VBATT_CHANNEL); 

    printf("finish module\n");

    /* チップの情報 */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("Chip: %s\tCores: %d\r\n",
           CONFIG_IDF_TARGET, chip_info.cores);

    while (1)
    {
        MICROMOUSE( adc, enc_R, enc_L, buzzer, imu, led, motor);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
