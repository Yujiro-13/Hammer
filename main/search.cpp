#include "include/Micromouse/UI/search.hpp"

void Search::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Search::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Search::ptr_by_control(t_control *_control) { control = _control; }

void Search::ptr_by_map(t_map *_map) { map = _map; }

void Search::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Search::ref_by_motion(Adachi &_adachi) { motion = _adachi;}

void Search::main_task()
{
    val->current.rad = 0.0;
    map->pos.x = 0;
    map->pos.y = 0;
    map->pos.dir = NORTH;
    control->log_flag = TRUE;
    motion.InitMaze();
    motion.search_adachi(map->GOAL_X,map->GOAL_Y);
    std::cout << "Search" << std::endl;
}

void All_Search::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void All_Search::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void All_Search::ptr_by_control(t_control *_control) { control = _control; }

void All_Search::ptr_by_map(t_map *_map) { map = _map; }

void All_Search::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void All_Search::ref_by_motion(Adachi &_adachi) { motion = _adachi;}

void All_Search::main_task()
{
    control->log_flag = TRUE;
    motion.search_adachi(5,5);
    std::cout << "All_Search" << std::endl;
}
