#include "include/Micromouse/Interrupt.hpp"
#include "include/Micromouse/UI/fast.hpp"
#include "include/Micromouse/UI/log.hpp"
#include "include/Micromouse/UI/search.hpp"
#include "include/Micromouse/UI/test.hpp"
#include "include/Micromouse/Motion/Adachi.hpp"
#include "include/Micromouse/Base_task.hpp"
#include <functional>

std::vector<std::shared_ptr<UI>> ui;
struct peripheral{
    ADC *adc;
    AS5047P *encR;
    AS5047P *encL;
    BUZZER *buzzer;
    MPU6500 *imu;
    PCA9632 *led;
    Motor *motor;
};

void MICROMOUSE(ADC &adc, AS5047P &enc_R, AS5047P &enc_L, BUZZER &buzzer, MPU6500 &imu, PCA9632 &led, Motor &motor);
void set_interface();
void call_task(UI *task, Adachi &motion);
void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map);
void mode_select(uint8_t *_mode_num, Adachi &adachi, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map, peripheral *periph);

void myTaskInterrupt(void *pvpram)
{
    Interrupt *interrupt = static_cast<Interrupt *>(pvpram);
    interrupt->interrupt();
}

void myTaskAdc(void *pvpram)
{
    ADC *adc = static_cast<ADC *>(pvpram);
    adc->adc_loop();
}

void myTaskLog(void *pvpram)
{
    Interrupt *log = static_cast<Interrupt *>(pvpram);
    log->logging();
}

/* 基本的に全ての処理のをここにまとめ、mainで呼び出す。 */

void MICROMOUSE(ADC &adc, AS5047P &enc_R, AS5047P &enc_L, BUZZER &buzzer, MPU6500 &imu, PCA9632 &led, Motor &motor)
{
    // printf("start MICROMOUSE\n");

    /* 構造体のインスタンス生成 */
    t_sens_data sens;
    t_mouse_motion_val val;
    t_control control;
    t_map map;
    t_file_pid_gain pid_gain;
    t_file_wall_th walll_threshold;
    t_file_center_sens_value center_sens_val;

    printf("finish struct\n");

    /* ログ取得用ハンドルの設定 */
    SemaphoreHandle_t on_logging = xSemaphoreCreateBinary();

    /* ポインタの設定・構造体の共有 */

    // 制御系
    Interrupt interrupt;
    interrupt.set_device(adc, enc_R, enc_L, buzzer, imu, led, motor);
    interrupt.ptr_by_sensor(&sens);
    interrupt.ptr_by_motion(&val);
    interrupt.ptr_by_control(&control);
    interrupt.ptr_by_map(&map);
    interrupt.GetSemphrHandle(&on_logging);

    printf("finish interrupt struct\n");

    // モーション系
    Adachi motion;
    motion.set_device(adc, enc_R, enc_L, buzzer, imu, led, motor);
    motion.ptr_by_sensor(&sens);
    motion.ptr_by_motion(&val);
    motion.ptr_by_control(&control);
    motion.ptr_by_map(&map);
    motion.GetSemphrHandle(&on_logging);

    //ペリフェラルまとめる
    peripheral periph;
    periph.adc = &adc;
    periph.encR = &enc_R;
    periph.encL = &enc_L;
    periph.buzzer = &buzzer;
    periph.imu = &imu;
    periph.led = &led;
    periph.motor = &motor;

    printf("finish motion struct\n");

    // センサ系
    adc.GetData(&sens);
    imu.GetData(&sens);
    enc_R.GetData(&sens);
    enc_L.GetData(&sens);

    printf("finish sensor struct\n");

    /* パラメータの設定 */
    pid_gain = read_file_pid();
    walll_threshold = read_file_wall_th();
    center_sens_val = read_file_center_sens_val();

    // 距離
    val.tar.len = 0.09;
    val.tar.len_half = 0.045;

    // 角度
    val.tar.rad = M_PI/2.0;

    // 速度
    //val.tar.acc = 0.5;
    val.max.acc = 1.0;
    //val.tar.vel = 0.3;
    val.max.vel = 0.3;
    val.min.vel = 0.05;
    val.end.vel = 0.3;

    // 角速度
    val.tar.ang_acc = 0.0;
    val.max.ang_acc = M_PI*5.0;
    val.tar.ang_vel = 0.0;
    val.max.ang_vel = M_PI;
    val.min.ang_vel = M_PI/5.0;
    val.end.ang_vel = 0.0;

    // 速度制御
    control.v.Kp = pid_gain.speed_Kp;
    control.v.Ki = pid_gain.speed_Ki;
    control.v.Kd = pid_gain.speed_Kd;

    // 角速度制御
    control.o.Kp = pid_gain.ang_vel_Kp;
    control.o.Ki = pid_gain.ang_vel_Ki;
    control.o.Kd = pid_gain.ang_vel_Kd;

    // 壁制御
    control.wall.Kp = pid_gain.wall_Kp;
    control.wall.Ki = pid_gain.wall_Ki;
    control.wall.Kd = pid_gain.wall_Kd;

    // 壁センサ閾値
    sens.wall.th_wall.fl = walll_threshold.th_wall_fl;
    sens.wall.th_wall.fr = walll_threshold.th_wall_fr;
    sens.wall.th_wall.l = walll_threshold.th_wall_l;
    sens.wall.th_wall.r = walll_threshold.th_wall_r;
    sens.wall.th_control.l = walll_threshold.th_control_l;
    sens.wall.th_control.r = walll_threshold.th_control_r;
    sens.wall.ref.l = walll_threshold.ref_l;
    sens.wall.ref.r = walll_threshold.ref_r;

    // ゴール座標
    map.GOAL_X = 4;
    map.GOAL_Y = 4;

    printf("finish parameter\n");
    // タスク優先順位 1 ~ 25    25が最高優先度
    xTaskCreatePinnedToCore(myTaskInterrupt,
                            "interrupt", 8192, &interrupt, configMAX_PRIORITIES, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(myTaskAdc,
                            "adc", 8192, &adc, configMAX_PRIORITIES - 1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(myTaskLog,
                            "log", 8192, &interrupt, configMAX_PRIORITIES - 2, NULL, APP_CPU_NUM);
    printf("finish task\n");

    /*char buffer[512];
    vTaskList(buffer);
    printf("Task execution statistics:\n%s", buffer);*/

    uint8_t mode = 0;
    const int MODE_MAX = 0b1111 - 1;
    const int MODE_MIN = 0;

    /* メインループ */
    while (1)
    {

        led.set(mode + 1);

        /*vTaskList(buffer);
        printf("Task execution statistics:\n%s", buffer);*/

        if (sens.wall.val.fl + sens.wall.val.l + sens.wall.val.r + sens.wall.val.fr > 3000)
        {

            led.set(0b1111);
            sens.gyro.ref = imu.surveybias(2000);
            mode_select(&mode, motion, &sens, &val, &control, &map, &periph);
            control.flag = FALSE;
        }

        if (val.current.vel > 0.04)
        {
            if (mode >= MODE_MAX)
            {
                mode = MODE_MIN;
            }
            else
            {
                mode++;
            }
            motor.setMotorSpeed(0.6, 0.6,0);
            vTaskDelay(pdMS_TO_TICKS(250));
            motor.setMotorSpeed(-0.6, -0.6,0);
            vTaskDelay(pdMS_TO_TICKS(250));
            motor.setMotorSpeed(0, 0,0);
        }
        if (val.current.vel < -0.04)
        {
            if (mode <= MODE_MIN)
            {
                mode = MODE_MAX;
            }
            else
            {
                mode--;
            }
            motor.setMotorSpeed(0.6, 0.6,0);
            vTaskDelay(pdMS_TO_TICKS(250));
            motor.setMotorSpeed(-0.6, -0.6,0);
            vTaskDelay(pdMS_TO_TICKS(250));
            motor.setMotorSpeed(0, 0,0);
        }
        //printf("time:%d\n", control.time_count);
        //printf("vel:%f\n", val.current.vel);
        //printf("rad:%f\n", val.current.rad);
        //printf("BatteryVoltage:%f\n", sens.BatteryVoltage);
        //printf("sens.wall.val.fl:%d sens.wall.val.l:%d sens.wall.val.r:%d sens.wall.val.fr:%d\n", sens.wall.val.fl, sens.wall.val.l, sens.wall.val.r, sens.wall.val.fr);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    //vTaskDelay(pdMS_TO_TICKS(10));
}

void set_interface()
{
    /* クラスのポインタを配列に保持*/

    ui.push_back(std::make_shared<Search>());
    ui.push_back(std::make_shared<All_Search>());
    ui.push_back(std::make_shared<Fast>());
    ui.push_back(std::make_shared<Fast2>());
    ui.push_back(std::make_shared<Fast3>());
    ui.push_back(std::make_shared<Fast4>());
    ui.push_back(std::make_shared<Test>());
    ui.push_back(std::make_shared<Test2>());
    ui.push_back(std::make_shared<Test3>());
    ui.push_back(std::make_shared<Test4>());
    ui.push_back(std::make_shared<Test5>());
    ui.push_back(std::make_shared<Test6>());
    ui.push_back(std::make_shared<Test7>());
    ui.push_back(std::make_shared<Log>());
    ui.push_back(std::make_shared<Log1>());

    //std::cout << "set_interface" << std::endl;
}

void call_task(UI *task, Adachi &motion, peripheral *periph)
{
    task->ref_by_motion(motion);
    task->set_device(*periph->adc, *periph->encR, *periph->encL, *periph->buzzer, *periph->imu, *periph->led, *periph->motor);
    task->main_task();
    //std::cout << "call_task" << std::endl;
}

void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map)
{
    task->ptr_by_sensor(_sen);
    task->ptr_by_motion(_val);
    task->ptr_by_control(_control);
    task->ptr_by_map(_map);
    //std::cout << "set_param" << std::endl;
}

void mode_select(uint8_t *_mode_num, Adachi &adachi, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map, peripheral *periph)
{
    set_interface();
    set_param(ui[*_mode_num].get(), sens, val, control, map);
    call_task(ui[*_mode_num].get(), adachi, periph);
    //std::cout << "mode_select" << std::endl;
}
