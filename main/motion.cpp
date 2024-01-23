#include "include/Micromouse/Motion/motion.hpp"

#define SECTION 0.09
#define SECTION_HALF 0.045
#define TURN_HALF M_PI
#define TURN_QUARTER M_PI/2.0

Motion::Motion() { /*std::cout << "Motion" << std::endl;*/ }

Motion::~Motion() {}

void Motion::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Motion::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Motion::ptr_by_control(t_control *_control) { control = _control; }

void Motion::ptr_by_map(t_map *_map) { map = _map; }

void Motion::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) {}

void Motion::GetSemphrHandle(SemaphoreHandle_t *_on_logging) { on_logging = _on_logging; }

void Motion::run()
{
    control->control_flag = TRUE;    // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    val->tar.len = SECTION;
    

    if (len_count == 7)
    {
        // val->tar.len = 45;
        val->tar.len = 0.045;
    }

    while (((val->tar.len - 0.03) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len) > val->current.len)
    {
        if (val->tar.vel <= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // val->tar.vel = val->tar.vel;
    val->tar.acc = 0.0;

    //std::cout << "run" << std::endl;
}

void Motion::run_half()
{
    control->control_flag = TRUE;    // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION_HALF;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;

    if (len_count == 7)
    {
        // val->tar.len = 45;
        val->tar.len = 0.045;
    }

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->tar.acc);

    while ((val->tar.len) > val->current.len)
    {
        if (val->tar.vel <= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // val->tar.vel = val->tar.vel;
    val->tar.acc = 0.0;

    //std::cout << "run" << std::endl;
}

void Motion::turn_left()
{
    vTaskDelay(100);

    control->control_flag = TRUE;    // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;        // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = val->max.ang_acc;
    val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_QUARTER;

    // std::cout << "turn_left" << std::endl;
    int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad > (val->current.rad - local_rad))
    {
        turn_count++;
        // printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    control->control_flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    //std::cout << "turn" << std::endl;
}

void Motion::turn_right()
{
    vTaskDelay(100);

    control->control_flag = TRUE;    // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = RIGHT;       // 右旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = -(val->max.ang_acc);
    //val->max.ang_vel = -(val->max.ang_vel);

    val->tar.rad = -(TURN_QUARTER);

    // std::cout << "turn_left" << std::endl;
    //int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad < (val->current.rad - local_rad))
    {
        //turn_count++;
        printf("val->tar.ang_vel : %f\n", val->tar.ang_vel);
        // printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    control->control_flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    //std::cout << "turn" << std::endl;
}

void Motion::turn_half()
{
    vTaskDelay(100);

    control->control_flag = TRUE;    // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;        // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    // std::cout << "turn_left" << std::endl;
    int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (M_PI > (val->current.rad - local_rad))
    {
        turn_count++;
        // printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    control->control_flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;
}

void Motion::stop()
{
    control->control_flag = TRUE;    // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.len = SECTION_HALF;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->tar.acc);

    while ((val->tar.len) > val->current.len)
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    control->control_flag = FALSE; // 制御OFF

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

void Motion::check_enkaigei()
{
    sens->wall.control = FALSE;
    control->control_flag = TRUE;

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    while (1)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    std::cout << "check_enkaigei" << std::endl;
}
