// #include "include/Micromouse/Micromouse.hpp"
#include "include/Micromouse/interrupt.hpp"
// #include "include/Micromouse/UI/UI.hpp"
#include "include/Micromouse/UI/fast.hpp"
#include "include/Micromouse/UI/log.hpp"
#include "include/Micromouse/UI/search.hpp"
#include "include/Micromouse/UI/test.hpp"
#include "include/Micromouse/Motion/motion.hpp"
#include "include/Micromouse/Motion/adachi.hpp"

std::vector<std::shared_ptr<UI>> ui;

void MICROMOUSE();
void set_interface();
void call_task(UI *task);
void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map);
void mode_select(uint8_t *_mode_num, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map);

/* 基本的に全ての処理のをここにまとめ、mainで呼び出す。 */

void MICROMOUSE()
{
    IRLED_FR LED_FR;
    IRLED_FL LED_FL;
    IRLED_R LED_R;
    IRLED_L LED_L;

    printf("finish IRLED\n");

    /* モジュールクラスのインスタンス生成と初期化 */
    ADC adc(LED_FR, LED_FL, LED_R, LED_L, VBATT_CHANNEL);
    AS5047P enc_R(SPI3_HOST, ENC_CS_R);
    AS5047P enc_L(SPI3_HOST, ENC_CS_L);
    BUZZER buzzer(BUZZER_CH, BUZZER_TIMER, BUZZER_PIN);
    MPU6500 imu(SPI2_HOST, IMU_CS);
    PCA9632 led(I2C_NUM_0, LED_ADRS);
    Motor motor(BDC_R_MCPWM_GPIO_PH, BDC_R_MCPWM_GPIO_EN, BDC_L_MCPWM_GPIO_PH, BDC_L_MCPWM_GPIO_EN, FAN_PIN);

    printf("finish module\n");

    /* 構造体のインスタンス生成 */
    t_sens_data sens;
    t_mouse_motion_val val;
    t_control control;
    t_map map;

    printf("finish struct\n");

    Interrupt interrupt;
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

    printf("finish interrupt\n");

    // モーション系
    Motion motion;
    motion.ptr_by_sensor(&sens);
    motion.ptr_by_motion(&val);
    motion.ptr_by_control(&control);
    motion.ptr_by_map(&map);

    printf("finish motion\n");

    // センサ系
    adc.GetData(&sens);
    imu.GetData(&sens);
    enc_R.GetData(&sens);
    enc_L.GetData(&sens);

    printf("mode: ");
    scanf("%hhd", &mode);

    while (1)
    {
        printf("Enter mode: ");
        scanf("%hhd", &mode);
        printf("\n");

        printf("Enter sens.wall.val.fl: ");
        scanf("%d", &sens.wall.val.fl);

        printf("Enter sens.wall.val.l: ");
        scanf("%d", &sens.wall.val.l);

        printf("Enter sens.wall.val.r: ");
        scanf("%d", &sens.wall.val.r);

        printf("Enter sens.wall.val.fr: ");
        scanf("%d", &sens.wall.val.fr);

        led.set(mode + 1);
        if (sens.wall.val.fl + sens.wall.val.l + sens.wall.val.r + sens.wall.val.fr > 3000)
        {

            led.set(0b1111);
            sens.gyro.ref = imu.surveybias(1000);
            mode_select(&mode, &sens, &val, &control, &map);
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
            // vTaskDelay(pdMS_TO_TICKS(100));
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
            // vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    // vTaskDelay(pdMS_TO_TICKS(10));
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

void call_task(UI *task)
{
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
    //
}

void mode_select(uint8_t *_mode_num, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map)
{
    set_interface();
    set_param(ui[*_mode_num].get(), sens, val, control, map);
    call_task(ui[*_mode_num].get());
    std::cout << "mode_select" << std::endl;
}
