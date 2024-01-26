#include "include/Micromouse/UI/log.hpp"

void Log::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Log::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Log::ptr_by_control(t_control *_control) { control = _control; }

void Log::ptr_by_map(t_map *_map) { map = _map; }

void Log::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Log::ref_by_motion(Adachi &_adachi) {}

void Log::log_print()
{
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "logs");
    if (partition == NULL)
    {
        ESP_LOGE("log", "Partition error");
        return;
    }

    uint32_t mem_offset = 0;
    int16_t data[10];

    while(1) {
        esp_partition_read(partition, mem_offset, data, sizeof(data));
        if(data[4] == -1){
            break;
        }
        printf("%4d,%4d,%4d,%4d,%4d,", data[0], data[1], data[2], data[3], data[4]);
        printf("%1d,%1d,%1d,%1d\n", data[5], data[6], data[7], data[8]);
        mem_offset += sizeof(data);
        if (mem_offset >= partition->size) {
            break;
        }
    }
    std::cout << "Log" << std::endl;
}

void Log::main_task()
{
    log_print();
    std::cout << "Log" << std::endl;
}

void Log1::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Log1::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Log1::ptr_by_control(t_control *_control) { control = _control; }

void Log1::ptr_by_map(t_map *_map) { map = _map; }

void Log1::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Log1::ref_by_motion(Adachi &_adachi) {}

void Log1::log_print()
{
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "logs");
    if (partition == NULL)
    {
        ESP_LOGE("log", "Partition error");
        return;
    }

    uint32_t mem_offset = 0;
    int16_t data[10];

    while(1) {
        esp_partition_read(partition, mem_offset, data, sizeof(data));
        if(data[4] == -1){
            break;
        }
        printf("%4d,%4d,%4d,%4d,%4d,", data[0], data[1], data[2], data[3], data[4]);
        printf("%1d,%1d,%1d,%1d\n", data[5], data[6], data[7], data[8]);
        mem_offset += sizeof(data);
        if (mem_offset >= partition->size) {
            break;
        }
    }
    std::cout << "Log" << std::endl;
}

void Log1::main_task()
{
    log_print();
    std::cout << "Log1" << std::endl;
}