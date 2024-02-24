#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "Micromouse/Base_task.hpp"
#include "include/Micromouse/Interrupt.hpp"
#include "include/files.hpp"

/*
    モード一覧
    1: 足立法片道探索
    2: 足立法全面探索（未実装）
    3: 足立法最短走行（未実装）
    4: 足立法最短走行（未実装）
    5: 足立法最短走行（未実装）
    6: 足立法最短走行（未実装）
    7: テスト1 宴会芸
    8: テスト2 直進確認
    9: テスト3 旋回確認（左90）
    10: テスト4 旋回確認（右90）
    11: テスト5 PIDゲイン・壁センサー閾値調整
    12: テスト6 直進停止確認
    13: テスト7 壁センサー確認
    14: ログ出力
    15: ログ出力
    16: ログ出力「マップ」（未実装）
*/


extern "C" void app_main(void)
{

    IRLED_FR LED_FR;
    IRLED_FL LED_FL;
    IRLED_R LED_R;
    IRLED_L LED_L;

    /* デバイスクラスのインスタンス生成と初期化 */ // 必ずmain関数内で行う

    Motor motor(BDC_R_MCPWM_GPIO_PH, BDC_R_MCPWM_GPIO_EN, BDC_L_MCPWM_GPIO_PH, BDC_L_MCPWM_GPIO_EN, FAN_PIN);
    PCA9632 led(I2C_NUM_0, LED_ADRS);
    BUZZER buzzer(BUZZER_PIN);
    AS5047P enc_R(SPI3_HOST, ENC_CS_R, false);
    AS5047P enc_L(SPI3_HOST, ENC_CS_L, true);
    MPU6500 imu(SPI2_HOST, IMU_CS);
    ADC adc(LED_FR, LED_FL, LED_R, LED_L, VBATT_CHANNEL);
    //NeoPixel neopixel(NEOPIXEL_PIN);

    printf("finish module\n");

    /* チップの情報 */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("Chip: %s\tCores: %d\r\n",
           CONFIG_IDF_TARGET, chip_info.cores);

    // motor.sincurve();
    //buzzer.play();
    static BUZZER::buzzer_score_t pc98[] = {
        {2000,100},{1000,100}
    };
    buzzer.play_melody(pc98, 2);
    /*
    for(uint32_t i=0;i<360;i++){
        neopixel.set_hsv({i, 100, 10});
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }
    neopixel.set_hsv({0, 0, 0});
    */

    /* ファイルシステムのマウント */
    init_files();

    while (1)
    {

        MICROMOUSE(adc, enc_R, enc_L, buzzer, imu, led, motor);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    unmount_fat();
}