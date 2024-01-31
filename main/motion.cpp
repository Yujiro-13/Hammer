#include "include/Micromouse/Motion/motion.hpp"

#define MODE_MAX 15
#define MODE_MIN 0
#define SECTION 0.09
#define SECTION_HALF 0.045
#define TURN_HALF M_PI
#define TURN_QUARTER M_PI / 2.0

Motion::Motion()
{ /*std::cout << "Motion" << std::endl;*/
}

Motion::~Motion() {}

void Motion::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Motion::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Motion::ptr_by_control(t_control *_control) { control = _control; }

void Motion::ptr_by_map(t_map *_map) { map = _map; }

void Motion::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot)
{
    adc = &_adc;
    encR = &_encR;
    encL = &_encL;
    buz = &_buz;
    imu = &_imu;
    led = &_led;
    mot = &_mot;
}

void Motion::GetSemphrHandle(SemaphoreHandle_t *_on_logging) { on_logging = _on_logging; }

void Motion::run()
{
    control->flag = TRUE;       // 制御ON
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

    // std::cout << "run" << std::endl;
}

void Motion::run_half()
{
    control->flag = TRUE;       // 制御ON
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

    // std::cout << "run" << std::endl;
}

void Motion::turn_left()
{
    vTaskDelay(100);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;   // 左旋回

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
    control->flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    // std::cout << "turn" << std::endl;
}

void Motion::turn_right()
{
    vTaskDelay(100);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = RIGHT;  // 右旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = -(val->max.ang_acc);
    // val->max.ang_vel = -(val->max.ang_vel);

    val->tar.rad = -(TURN_QUARTER);

    // std::cout << "turn_left" << std::endl;
    // int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad < (val->current.rad - local_rad))
    {
        // turn_count++;
        // printf("val->tar.ang_vel : %f\n", val->tar.ang_vel);
        //  printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    control->flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    // std::cout << "turn" << std::endl;
}

void Motion::turn_half()
{
    vTaskDelay(100);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;   // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = val->max.ang_acc;
    val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_HALF;

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

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    while (val->current.ang_vel >= 0.01 || val->current.ang_vel <= -0.01)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE;

    // std::cout << "turn" << std::endl;
}

void Motion::stop()
{
    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION_HALF;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel)) / (2.0 *
                                                                                              val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len) > val->current.len)
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.acc = 0.0;
    val->tar.vel = 0.0;

    while (val->current.vel >= 0.0)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE; // 制御OFF

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
    control->flag = TRUE;

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

void Motion::turn_left_2()
{
    vTaskDelay(100);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;   // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = val->max.ang_acc;
    // val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_QUARTER;

    // std::cout << "turn_left" << std::endl;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad - (val->current.rad - local_rad) > (val->tar.ang_vel * val->tar.ang_vel) / (2.0 * val->tar.ang_acc))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = -(val->max.ang_acc);

    while (val->tar.rad > (val->current.rad - local_rad))
    {
        if (val->tar.ang_vel < val->min.ang_vel)
        {
            val->tar.ang_acc = 0;
            val->tar.ang_vel = val->min.ang_vel;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    while (val->current.ang_vel >= 0.01 || val->current.ang_vel <= -0.01)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE;

    // std::cout << "turn" << std::endl;
}

void Motion::turn_right_2()
{
    vTaskDelay(100);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = RIGHT;  // 右旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = -(val->max.ang_acc);
    // val->max.ang_vel = -(val->max.ang_vel);

    val->tar.rad = -(TURN_QUARTER);

    // std::cout << "turn_left" << std::endl;
    // int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (-(val->tar.rad - (val->current.rad - local_rad)) > (val->tar.ang_vel * val->tar.ang_vel) / (2.0 * -(val->tar.ang_acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = val->max.ang_acc;

    while (val->tar.rad < (val->current.rad - local_rad))
    {
        if (val->tar.ang_vel > -(val->min.ang_vel))
        {
            val->tar.ang_acc = 0;
            val->tar.ang_vel = -(val->min.ang_vel);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    // std::cout << "##### deceleration #####" << std::endl;
    while (val->current.ang_vel >= 0.01 || val->current.ang_vel <= -0.01)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE;

    // std::cout << "turn" << std::endl;
}

void Motion::wall_check()
{
    sens->wall.control = FALSE;
    control->flag = FALSE;

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    while (1)
    {
        printf("sens.wall.val.fl:%d    sens.wall.val.l:%d    sens.wall.val.r:%d    sens.wall.val.fr:%d\n", sens->wall.val.fl, sens->wall.val.l, sens->wall.val.r, sens->wall.val.fr);
        led->set(sens->wall.exist.fr + (sens->wall.exist.r << 1) + (sens->wall.exist.l << 2) + (sens->wall.exist.fl << 3));
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    std::cout << "check_enkaigei" << std::endl;
}

void Motion::adjust_pid(const char *gain, float *pid, float step, uint8_t mode_num)
{

    while (1)
    {
        led->set(mode_num + 1);

        if (sens->wall.val.fl + sens->wall.val.l + sens->wall.val.r + sens->wall.val.fr > 3000)
        {
            led->set(0b1111);
            printf("adjust %s\n", gain);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            return;
        }

        if (val->current.vel > 0.02)
        {
            if (mode_num >= MODE_MAX)
            {
                mode_num = MODE_MIN;
            }
            else
            {
                *pid += step;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (val->current.vel < -0.02)
        {
            if (mode_num <= MODE_MIN)
            {
                mode_num = MODE_MAX;
            }
            else
            {
                *pid -= step;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        printf("%s : %f\n", gain, *pid);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Motion::set_pid_gain()
{
    uint8_t mode = 0;

    const char *speed_Kp = "speed_Kp";
    const char *speed_Ki = "speed_Ki";
    const char *speed_Kd = "speed_Kd";
    const char *ang_vel_Kp = "ang_vel_Kp";
    const char *ang_vel_Ki = "ang_vel_Ki";
    const char *ang_vel_Kd = "ang_vel_Kd";
    const char *wall_Kp = "wall_Kp";
    const char *wall_Ki = "wall_Ki";
    const char *wall_Kd = "wall_Kd";

    t_file_pid_gain pid_gain = read_file_pid();

    adjust_pid(speed_Kp, &pid_gain.speed_Kp, 0.1, mode);
    adjust_pid(speed_Ki, &pid_gain.speed_Ki, 0.1, mode + 1);
    adjust_pid(speed_Kd, &pid_gain.speed_Kd, 0.1, mode + 2);
    adjust_pid(ang_vel_Kp, &pid_gain.ang_vel_Kp, 0.1, mode + 3);
    adjust_pid(ang_vel_Ki, &pid_gain.ang_vel_Ki, 0.1, mode + 4);
    adjust_pid(ang_vel_Kd, &pid_gain.ang_vel_Kd, 0.1, mode + 5);
    adjust_pid(wall_Kp, &pid_gain.wall_Kp, 0.1, mode + 6);
    adjust_pid(wall_Ki, &pid_gain.wall_Ki, 0.1, mode + 7);
    adjust_pid(wall_Kd, &pid_gain.wall_Kd, 0.1, mode + 8);

    write_file_pid(&pid_gain);

    printf("set_pid_gain\n");
}

void Motion::adjust_wall_threshold(const char *threshold, uint16_t *th_value, uint8_t step, uint8_t mode_num)
{

    while (1)
    {
        led->set(mode_num + 1);

        if (sens->wall.val.fl + sens->wall.val.l + sens->wall.val.r + sens->wall.val.fr > 3000)
        {
            led->set(0b1111);
            printf("adjust %s\n", threshold);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            return;
        }

        if (val->current.vel > 0.02)
        {
            if (mode_num >= MODE_MAX)
            {
                mode_num = MODE_MIN;
            }
            else
            {
                *th_value += step;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (val->current.vel < -0.02)
        {
            if (mode_num <= MODE_MIN)
            {
                mode_num = MODE_MAX;
            }
            else
            {
                *th_value -= step;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        printf("%s : %d\n", threshold, *th_value);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Motion::set_wall_threshold()
{
    uint8_t mode = 0;

    const char *th_wall_fl = "th_wall_fl";
    const char *th_wall_l = "th_wall_l";
    const char *th_wall_r = "th_wall_r";
    const char *th_wall_fr = "th_wall_fr";
    const char *th_control_l = "th_control_l";
    const char *th_control_r = "th_control_r";
    const char *ref_l = "ref_l";
    const char *ref_r = "ref_r";

    t_file_wall_th th_value = read_file_wall_th();

    adjust_wall_threshold(th_wall_fl, &th_value.th_wall_fl, 1, mode);
    adjust_wall_threshold(th_wall_l, &th_value.th_wall_l, 1, mode + 1);
    adjust_wall_threshold(th_wall_r, &th_value.th_wall_r, 1, mode + 2);
    adjust_wall_threshold(th_wall_fr, &th_value.th_wall_fr, 1, mode + 3);
    adjust_wall_threshold(th_control_l, &th_value.th_control_l, 1, mode + 4);
    adjust_wall_threshold(th_control_r, &th_value.th_control_r, 1, mode + 5);
    adjust_wall_threshold(ref_l, &th_value.ref_l, 1, mode + 6);
    adjust_wall_threshold(ref_r, &th_value.ref_r, 1, mode + 7);

    write_file_wall_th(&th_value);

    printf("set_pid_gain\n");
}