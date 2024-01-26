#ifndef INTERRUPT_HPP
#define INTERRUPT_HPP

#include <iostream>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>    // freertos以下のファイルをインクルードしたい場合、必ず先にFreeRTOS.hをインクルードする
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "spi_flash_mmap.h"
#include "../structs.hpp"
#include "Micromouse.hpp"


class Interrupt : public Micromouse{
    public:
        Interrupt();
        ~Interrupt();
        void interrupt();
        void ptr_by_sensor(t_sens_data *sens) override;
        void ptr_by_motion(t_mouse_motion_val *val) override;
        void ptr_by_control(t_control *control) override;
        void ptr_by_map(t_map *map) override;
        void set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void GetSemphrHandle(SemaphoreHandle_t *_on_logging);
        void reset_I_gain();
        void logging();
    private:
        void calc_target();
        void wall_control();
        void feedback_control();
        void calc_distance();
        void calc_angle();
        t_sens_data *sens;
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        float _accel = 0.0;
        float _vel = 0.0;
        float max_vel = 0.0;
        float max_ang_vel = 0.0;

        ADC *adc;
        AS5047P *encR;
        AS5047P *encL;
        BUZZER *buz;
        MPU6500 *imu;
        PCA9632 *led;
        Motor *mot;

        SemaphoreHandle_t *on_logging;




};





#endif // INTERRUPT_HPP