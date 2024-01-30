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
#include "MIcromouse/Base_task.hpp"
#include "include/MIcromouse/interrupt.hpp"
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
    11: テスト5 旋回確認（左180度）
    12: テスト6 直進停止確認
    13: テスト7 壁センサー確認
    14: ログ出力
    15: ログ出力
    16: ログ出力「マップ」（未実装）
*/

int target_param = 0;

extern "C" void app_main(void)
{

    IRLED_FR LED_FR;
    IRLED_FL LED_FL;
    IRLED_R LED_R;
    IRLED_L LED_L;

    /* デバイスクラスのインスタンス生成と初期化 */ // 必ずmain関数内で行う

    Motor motor(BDC_R_MCPWM_GPIO_PH, BDC_R_MCPWM_GPIO_EN, BDC_L_MCPWM_GPIO_PH, BDC_L_MCPWM_GPIO_EN, FAN_PIN);
    PCA9632 led(I2C_NUM_0, LED_ADRS);
    BUZZER buzzer(BUZZER_CH, BUZZER_TIMER, BUZZER_PIN);
    AS5047P enc_R(SPI3_HOST, ENC_CS_R, false);
    AS5047P enc_L(SPI3_HOST, ENC_CS_L, true);
    MPU6500 imu(SPI2_HOST, IMU_CS);
    ADC adc(LED_FR, LED_FL, LED_R, LED_L, VBATT_CHANNEL);

    printf("finish module\n");

    /* チップの情報 */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("Chip: %s\tCores: %d\r\n",
           CONFIG_IDF_TARGET, chip_info.cores);

    // motor.sincurve();

    /*const char *TAG = "file_example";
    const char *PARTITION_LABEL = "storage";
    const std::string BASE_PATH = "/param";
    const std::string FILE_PATH = BASE_PATH + "/pid.txt";
    const esp_vfs_fat_mount_config_t MOUNT_CONFIG = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };

    wl_handle_t wl_handle = WL_INVALID_HANDLE;

    esp_err_t err = esp_vfs_fat_spiflash_mount(
        BASE_PATH.c_str(),
        PARTITION_LABEL,
        &MOUNT_CONFIG,
        &wl_handle
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Opening file");
    std::ifstream ifile(FILE_PATH, std::ios::in | std::ios::binary);
    if(ifile.fail())
    {
        ESP_LOGE(TAG, "Failed to open %s for reading", FILE_PATH.c_str());
        //return;
    }else{
        std::string str;
        getline(ifile, str);
        //target_param = std::stoi(str);

        ESP_LOGI(TAG, "target_param: %d", target_param);
    }
    ifile.close();

    ESP_LOGI(TAG, "write file : %d", target_param);
    std::ofstream ffile(FILE_PATH, std::ios::out | std::ios::binary);
    if(ffile.fail())
    {
        ESP_LOGE(TAG, "Failed to open %s for writing", FILE_PATH.c_str());
        //return;
    }else{
        ffile << target_param + 1 << std::endl;
        ffile.close();
        ESP_LOGI(TAG, "write file : %d", target_param + 1);
    }

    ESP_LOGI(TAG, "Umounting FATFS");
    esp_vfs_fat_spiflash_unmount(BASE_PATH.c_str(), wl_handle);*/

    init_files();

    while (1)
    {

        MICROMOUSE(adc, enc_R, enc_L, buzzer, imu, led, motor);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    unmount_fat();
}
