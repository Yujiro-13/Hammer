#include "include/Micromouse/interrupt.hpp"
#include "include/Micromouse/UI/fast.hpp"
#include "include/Micromouse/UI/log.hpp"
#include "include/Micromouse/UI/search.hpp"
#include "include/Micromouse/UI/test.hpp"
// #include "include/Micromouse/Motion/motion.hpp"
#include "include/Micromouse/Motion/adachi.hpp"
#include <functional>

std::vector<std::shared_ptr<UI>> ui;

void MICROMOUSE(ADC &adc, AS5047P &enc_R, AS5047P &enc_L, BUZZER &buzzer, MPU6500 &imu, PCA9632 &led, Motor &motor, Interrupt &interrupt);
void set_interface();
void call_task(UI *task, Adachi &motion);
void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map);
void mode_select(uint8_t *_mode_num, Adachi &adachi, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map);

/* 基本的に全ての処理のをここにまとめ、mainで呼び出す。 */



void MICROMOUSE(ADC &adc, AS5047P &enc_R, AS5047P &enc_L, BUZZER &buzzer, MPU6500 &imu, PCA9632 &led, Motor &motor, Interrupt &interrupt)
{

    //printf("start MICROMOUSE\n");

    //printf("finish IRLED\n");

    

    printf("finish module\n");

    /* 構造体のインスタンス生成 */
    t_sens_data sens;
    t_mouse_motion_val val;
    t_control control;
    t_map map;

    printf("finish struct\n");

    uint8_t mode = 0;
    const int MODE_MAX = 0b0111;
    const int MODE_MIN = 0;

    /* ポインタの設定・構造体の共有 */

    // 制御系

    interrupt.set_module(adc, enc_R, enc_L, buzzer, imu, led, motor);
    interrupt.ptr_by_sensor(&sens);
    interrupt.ptr_by_motion(&val);
    interrupt.ptr_by_control(&control);
    interrupt.ptr_by_map(&map);

    printf("finish interrupt struct\n");

    // モーション系
    Adachi motion;
    motion.ptr_by_sensor(&sens);
    motion.ptr_by_motion(&val);
    motion.ptr_by_control(&control);
    motion.ptr_by_map(&map);

    printf("finish motion struct\n");

    // センサ系
    adc.GetData(&sens);
    imu.GetData(&sens);
    enc_R.GetData(&sens);
    enc_L.GetData(&sens);

    printf("finish sensor struct\n");

    /* パラメータの設定 */

    // 距離
    val.tar.len = 0.09;
    val.tar.len_half = 0.045;

    // 速度
    val.tar.acc = 0.5;
    val.tar.vel = 0.5;
    val.max.vel = 0.5;
    val.min.vel = 0.1;
    val.end.vel = 0.0;

    // 角速度
    val.tar.ang_acc = 0.5;
    val.tar.ang_vel = 0.5;
    val.max.ang_vel = 0.5;
    val.min.ang_vel = 0.1;
    val.end.ang_vel = 0.0;

    // 速度制御
    control.v.Kp = 0.5;
    control.v.Ki = 0.0;
    control.v.Kd = 0.0;

    // 角速度制御
    control.o.Kp = 0.5;
    control.o.Ki = 0.0;
    control.o.Kd = 0.0;

    // 壁制御
    control.wall.Kp = 0.5;
    control.wall.Ki = 0.0;
    control.wall.Kd = 0.0;

    // 壁センサ閾値
    sens.wall.th_wall.fl = 42;
    sens.wall.th_wall.fr = 47;
    sens.wall.th_wall.l = 41;
    sens.wall.th_wall.r = 55;
    sens.wall.th_control.l = 100;
    sens.wall.th_control.r = 100;
    sens.wall.ref.l = 141;
    sens.wall.ref.r = 170;

    printf("finish parameter\n");

    

    /* メインループ */
    while (1)
    {

        led.set(mode + 1);
        if (sens.wall.val.fl + sens.wall.val.l + sens.wall.val.r + sens.wall.val.fr > 3000)
        {

            led.set(0b1111);
            sens.gyro.ref = imu.surveybias(1000);
            mode_select(&mode, motion, &sens, &val, &control, &map);
            control.control_flag = FALSE;
            break;
        }

        if (val.current.vel > 0.05)
        {
            if (mode >= MODE_MAX)
            {
                mode = MODE_MIN;
            }
            else
            {
                mode++;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (val.current.vel < -0.05)
        {
            if (mode <= MODE_MIN)
            {
                mode = MODE_MAX;
            }
            else
            {
                mode--;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
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
    ui.push_back(std::make_shared<Log>());
    ui.push_back(std::make_shared<Log1>());

    std::cout << "set_interface" << std::endl;
}

void call_task(UI *task, Adachi &motion)
{
    task->ref_by_motion(motion);
    task->main_task();
    std::cout << "call_task" << std::endl;
}

void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map)
{
    task->ptr_by_sensor(_sen);
    task->ptr_by_motion(_val);
    task->ptr_by_control(_control);
    task->ptr_by_map(_map);
    std::cout << "set_param" << std::endl;
}

void mode_select(uint8_t *_mode_num, Adachi &adachi, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map)
{
    set_interface();
    set_param(ui[*_mode_num].get(), sens, val, control, map);
    call_task(ui[*_mode_num].get(), adachi);
    std::cout << "mode_select" << std::endl;
}
