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
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
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
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
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
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
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
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
};

class Test5 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
};

class Test6 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
};

class Test7 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
        void ref_by_motion(Adachi &_adachi) override;
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
};

#endif // TEST_HPP