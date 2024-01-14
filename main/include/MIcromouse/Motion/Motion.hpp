#ifndef MOTION_HPP
#define MOTION_HPP

#include <iostream>
#include "../Micromouse.hpp"
//#include "../Micromouse/interrupt.hpp"

class Motion : Micromouse
{
    public:
        Motion();
        ~Motion();
        void ptr_by_sensor(t_sens_data *sens) override;
        void ptr_by_motion(t_mouse_motion_val *val) override;
        void ptr_by_control(t_control *control) override;
        void ptr_by_map(t_map *map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void run();
        void turn();
        void stop();
        void back();
        void slalom();
    protected:
        t_sens_data *sens;   // 後でexternの方を消し、こっちに書き換える。ここは、privateかprotected要検討
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
    private:
        //Interrupt *interrupt;
        uint8_t len_count = 0;
        float local_rad = 0.0;
};

#endif // MOTION_HPP