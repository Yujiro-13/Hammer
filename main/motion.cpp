#include "include/Micromouse/Motion/motion.hpp"

Motion::Motion(){std::cout << "Motion" << std::endl;}

Motion::~Motion(){}

void Motion::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Motion::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Motion::ptr_by_control(t_control *_control) { control = _control; }

void Motion::ptr_by_map(t_map *_map) { map = _map; }

void Motion::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Motion::run()
{
    control->control_flag = TRUE; // 制御ON
    sens->wall.wall_control = FALSE; // 壁制御OFF

    //val->max.vel = val->max.vel; // 目標（最大）速度設定
    //val->current.acc = val->acc; // 加速度設定
    //float end_vel = val->end.vel;

    //interrupt->reset_I_gain(); // 積分値リセット
    val->current.len = 0.0;

	if (len_count == 7)
	{
		//val->tar.len = 45;
		val->tar.len = 0.045;
	}else{
		//val->tar.len = val->tar.len;
	}
	
	
    while (((val->tar.len - 0.03) - val->current.len) > (((val->tar.vel)*(val->tar.vel) - (val->end.vel)*(val->end.vel)) / (2.0 * 
    val->tar.acc)))
    {
        //vTaskDelay(1);
    }

    //std::cout << "##### deceleration #####" << std::endl;
    val->current.acc = -(val->tar.acc);

    while ((val->tar.len) > val->current.len)
    {
        if (val->tar.vel <= val->max.vel)
        {
            val->current.acc = 0;
            val->tar.vel = val->max.vel;
        }
        //vTaskDelay(1);
        
    }
    
    val->tar.vel = val->tar.vel;
    val->current.acc = 0.0;

    std::cout << "run" << std::endl;
}

void Motion::turn()
{
    //vTaskDelay(100);
    
    control->control_flag = TRUE; // 制御ON
    sens->wall.wall_control = FALSE; // 壁制御OFF
    val->current.flag = LEFT; // 左旋回

    val->max.ang_vel = val->tar.ang_vel; // 目標（最大）角速度設定
    val->current.ang_acc = val->tar.ang_acc; // 角加速度設定

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    //interrupt->reset_I_gain(); // 積分値リセット
    
    //std::cout << "turn_left" << std::endl;
    int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存
    
    while(M_PI / 2 > (val->current.rad - local_rad)){
        turn_count++;
        //printf("turn_count : %d\n", turn_count);
        //vTaskDelay(1);
    }

    //std::cout << "##### deceleration #####" << std::endl;
    control->control_flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->current.ang_acc = 0.0;

    std::cout << "turn" << std::endl;
}

void Motion::stop()
{
    std::cout << "stop" << std::endl;
}

void Motion::back()
{
    std::cout << "back" << std::endl;
}

void Motion::slalom()
{
    std::cout << "slalom" << std::endl;
}

