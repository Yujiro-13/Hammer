#include "include/Micromouse/UI/test.hpp"

void Test::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test::ptr_by_control(t_control *_control) { control = _control; }

void Test::ptr_by_map(t_map *_map) { map = _map; }

void Test::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}


void Test::main_task()
{
    std::cout << "Test" << std::endl;
}


void Test2::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test2::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test2::ptr_by_control(t_control *_control) { control = _control; }

void Test2::ptr_by_map(t_map *_map) { map = _map; }

void Test2::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Test2::main_task()
{
    std::cout << "Test2" << std::endl;
}

void Test3::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test3::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test3::ptr_by_control(t_control *_control) { control = _control; }

void Test3::ptr_by_map(t_map *_map) { map = _map; }

void Test3::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Test3::main_task()
{
    std::cout << "Test3" << std::endl;
}

void Test4::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test4::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test4::ptr_by_control(t_control *_control) { control = _control; }

void Test4::ptr_by_map(t_map *_map) { map = _map; }

void Test4::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Test4::main_task()
{
    std::cout << "Test4" << std::endl;
}

