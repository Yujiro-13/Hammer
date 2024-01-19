#include <stdio.h>
#include <iostream>
#include <fstream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "MIcromouse/Base_task.hpp"
#include "include/MIcromouse/interrupt.hpp"
//#include "include/Micromouse/UI/UI.hpp"







extern "C" void app_main(void)
{
    
    IRLED_FR LED_FR;
    IRLED_FL LED_FL;
    IRLED_R LED_R;
    IRLED_L LED_L;
    /* デバイスクラスのインスタンス生成と初期化 デバイスの初期化はmain関数内で行わなければならない*/ 
    Motor motor(BDC_R_MCPWM_GPIO_PH, BDC_R_MCPWM_GPIO_EN, BDC_L_MCPWM_GPIO_PH, BDC_L_MCPWM_GPIO_EN, FAN_PIN); // I (420) gpio: GPIO[40]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 Guru Meditation Error: Core  0 panic'ed (StoreProhibited). Exception was unhandled.
    /*Inter-Processor Call (IPC) task stack size 1280->2560で解決*/
    PCA9632 led(I2C_NUM_0, LED_ADRS);                   // ***ERROR*** A stack overflow in task main has been detected.
    BUZZER buzzer(BUZZER_CH, BUZZER_TIMER, BUZZER_PIN); // NO error
    AS5047P enc_R(SPI3_HOST, ENC_CS_R,false);                 // encoder init 連続で呼び出すとエラー
    AS5047P enc_L(SPI3_HOST, ENC_CS_L,true);                 // Guru Meditation Error: Core  0 panic'ed (StoreProhibited). Exception was unhandled. or assert failed: xQueueSemaphoreTake queue.c:1657 (pxQueue->uxItemSize == 0)
    MPU6500 imu(SPI2_HOST, IMU_CS);                     // ***ERROR*** A stack overflow in task main has been detected.
    ADC adc(LED_FR, LED_FL, LED_R, LED_L, VBATT_CHANNEL); // Guru Meditation Error: Core  0 panic'ed (LoadProhibited). Exception was unhandled

    Interrupt interrupt;

    printf("finish interrupt\n");

    

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("Chip: %s\tCores: %d\r\n",
           CONFIG_IDF_TARGET, chip_info.cores);

    


    

    //printf("finish task\n");

    while (1)
    {
        MICROMOUSE( adc, enc_R, enc_L, buzzer, imu, led, motor, interrupt);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
