#ifndef TEST_HPP
#define TEST_HPP

//#include <iostream>
#include "UI.hpp"

class Test : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
    private:
        t_sens_data *sens;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
};

class Test2 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
    private:
        t_sens_data *sens;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
};

class Test3 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
    private:
        t_sens_data *sens;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
};

class Test4 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
    private:
        t_sens_data *sens;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
};

#endif // TEST_HPP