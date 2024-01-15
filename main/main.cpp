#include <stdio.h>
#include <iostream>
#include <fstream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "include/Micromouse/Micromouse.hpp"
#include "include/MIcromouse/interrupt.hpp"


extern "C" void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("Chip: %s\tCores: %d\r\n",
        CONFIG_IDF_TARGET, chip_info.cores);

    

    
    while (1)
    {
        MICROMOUSE();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}
