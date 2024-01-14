#ifndef INTERRUPT_HPP
#define INTERRUPT_HPP

#include <iostream>
#include <string.h>
#include "../structs.hpp"
#include "Micromouse.hpp"


class Interrupt : public Micromouse{
    public:
        Interrupt();
        ~Interrupt();
        void interrupt(void *pvparam);
        void ptr_by_sensor(t_sens_data *sens) override;
        void ptr_by_motion(t_mouse_motion_val *val) override;
        void ptr_by_control(t_control *control) override;
        void ptr_by_map(t_map *map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void reset_I_gain();
    private:
        void calc_target();
        void wall_control();
        void feedback_control();
        void calc_distance();
        void calc_angle();
        t_sens_data *sens;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        float _accel = 0.0;
        float _vel = 0.0;

        ADC *adc;
        AS5047P *encR;
        AS5047P *encL;
        BUZZER *buz;
        MPU6500 *imu;
        PCA9632 *led;
        Motor *mot;

        t_sensing_result result;




};



#endif // INTERRUPT_HPP