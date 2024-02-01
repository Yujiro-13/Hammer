#include "include/Micromouse/UI/test.hpp"



void Test::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test::ptr_by_control(t_control *_control) { control = _control; }

void Test::ptr_by_map(t_map *_map) { map = _map; }

void Test::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test::main_task()
{   
    control->log_flag = TRUE;
    motion.check_enkaigei(); // ok
    control->log_flag = FALSE;
    std::cout << "Test" << std::endl;
}

void Test2::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test2::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test2::ptr_by_control(t_control *_control) { control = _control; }

void Test2::ptr_by_map(t_map *_map) { map = _map; }

void Test2::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test2::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test2::main_task()
{
    control->log_flag = TRUE;
    motion.run();
    control->log_flag = FALSE;
    std::cout << "Test2" << std::endl;
}

void Test3::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test3::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test3::ptr_by_control(t_control *_control) { control = _control; }

void Test3::ptr_by_map(t_map *_map) { map = _map; }

void Test3::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test3::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test3::main_task()
{
    control->log_flag = TRUE;
    motion.turn_left_2(); // ok
    control->log_flag = FALSE;
    std::cout << "Test3" << std::endl;
}

void Test4::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test4::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test4::ptr_by_control(t_control *_control) { control = _control; }

void Test4::ptr_by_map(t_map *_map) { map = _map; }

void Test4::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test4::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test4::main_task()
{
    control->log_flag = TRUE;
    motion.turn_right_2(); // ok
    control->log_flag = FALSE;
    std::cout << "Test4" << std::endl;
}

void Test5::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test5::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test5::ptr_by_control(t_control *_control) { control = _control; }

void Test5::ptr_by_map(t_map *_map) { map = _map; }

void Test5::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test5::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test5::main_task()
{   
    motion.set_pid_gain();
    motion.set_wall_threshold();
    std::cout << "Test" << std::endl;
}

void Test6::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test6::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test6::ptr_by_control(t_control *_control) { control = _control; }

void Test6::ptr_by_map(t_map *_map) { map = _map; }

void Test6::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test6::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test6::main_task()
{
    control->log_flag = TRUE;
    motion.stop();  // OK
    control->log_flag = FALSE;
    std::cout << "Test2" << std::endl;
}

void Test7::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Test7::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Test7::ptr_by_control(t_control *_control) { control = _control; }

void Test7::ptr_by_map(t_map *_map) { map = _map; }

void Test7::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Test7::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Test7::main_task()
{
    control->log_flag = TRUE;
    motion.wall_check(); // OK
    control->log_flag = FALSE;
    std::cout << "Test3" << std::endl;
}

