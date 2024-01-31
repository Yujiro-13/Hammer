#include "include/Micromouse/interrupt.hpp"

#define TIRE_DIAMETER 0.01368
#define MMPP TIRE_DIAMETER*M_PI / ENC_MAX



Interrupt::Interrupt()
{ /*std::cout << "Interrupt" << std::endl;*/
}

Interrupt::~Interrupt() { std::cout << "~Interrupt" << std::endl; }

void Interrupt::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Interrupt::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Interrupt::ptr_by_control(t_control *_control) { control = _control; }

void Interrupt::ptr_by_map(t_map *_map) { map = _map; }

void Interrupt::set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot)
{
    adc = &_adc;
    encR = &_encR;
    encL = &_encL;
    buz = &_buz;
    imu = &_imu;
    led = &_led;
    mot = &_mot;
    std::cout << "set_device" << std::endl;
}

void Interrupt::GetSemphrHandle(SemaphoreHandle_t *_on_logging) { on_logging = _on_logging; }

void Interrupt::calc_target()
{ //  目標値を計算する

    val->tar.vel += (val->tar.acc) / 1000.0;

    if (val->tar.vel > val->max.vel)
    {
        val->tar.vel = val->max.vel;
    }

    val->tar.ang_vel += (val->tar.ang_acc) / 1000.0;

    if (val->current.flag == LEFT)
    {

        if (val->tar.ang_vel > val->max.ang_vel)
        {
            val->tar.ang_vel = val->max.ang_vel;
        }
    }
    else if (val->current.flag == RIGHT)
    {

        if (val->tar.ang_vel < -(val->max.ang_vel))
        {
            val->tar.ang_vel = -(val->max.ang_vel);
        }
    }

    return;
}

void Interrupt::wall_control() //  壁制御
{
    // 左前壁センサ
    if (sens->wall.val.fl > sens->wall.th_wall.fl)
    {
        sens->wall.exist.fl = TRUE;
    }
    else
    {
        sens->wall.exist.fl = FALSE;
    }

    // 右前壁センサ
    if (sens->wall.val.fr > sens->wall.th_wall.fr)
    {
        sens->wall.exist.fr = TRUE;
    }
    else
    {
        sens->wall.exist.fr = FALSE;
    }

    // 左壁センサ
    if (sens->wall.val.l > sens->wall.th_wall.l)
    {
        sens->wall.exist.l = TRUE;
    }
    else
    {
        sens->wall.exist.l = FALSE;
    }
    // 右壁センサ
    if (sens->wall.val.r > sens->wall.th_wall.r)
    {
        sens->wall.exist.r = TRUE;
    }
    else
    {
        sens->wall.exist.r = FALSE;
    }

    if (sens->wall.val.l > sens->wall.th_control.l)
    {
        sens->wall.error.l = sens->wall.val.l - sens->wall.ref.l;
        sens->wall.control_enable.l = TRUE;
    }
    else
    {
        sens->wall.error.l = 0;
        sens->wall.control_enable.l = FALSE;
    }

    if (sens->wall.val.r > sens->wall.th_control.r)
    {
        sens->wall.error.r = sens->wall.val.r - sens->wall.ref.r;
        sens->wall.control_enable.r = TRUE;
    }
    else
    {
        sens->wall.error.r = 0;
        sens->wall.control_enable.r = FALSE;
    }

    if (sens->wall.control == TRUE && sens->wall.val.fl + sens->wall.val.fr <= (sens->wall.th_wall.fl + sens->wall.th_wall.fr) * 5.0)
    {

        if (sens->wall.control_enable.l == TRUE && sens->wall.control_enable.r == TRUE)
        {
            val->current.wall_error = sens->wall.error.r - sens->wall.error.l;
        }
        else if (sens->wall.control_enable.l == FALSE && sens->wall.control_enable.r == TRUE)
        {
            val->current.wall_error = sens->wall.error.r;
        }
        else if (sens->wall.control_enable.l == TRUE && sens->wall.control_enable.r == FALSE)
        {
            val->current.wall_error = -(sens->wall.error.l);
        }
        else
        {
            val->current.wall_error = 0;
        }

        val->I.wall_error += val->current.wall_error / 1000.0;
        val->p.wall_error = (val->p.wall_error - val->current.wall_error) * 1000.0;

        val->tar.wall_val = val->current.wall_error * (control->wall.Kp) + val->I.wall_error * (control->wall.Ki) - val->p.wall_error * (control->wall.Kd);
        val->tar.ang_vel = val->tar.wall_val;

        val->p.wall_error = val->current.wall_error;
    }
    // xSemaphoreGive(on_logging);

    // std::cout << "wall_ctl" << std::endl;
    return;
}

void Interrupt::feedback_control()
{ // フィードバック制御
    if (control->flag == TRUE)
    {
        // 速度制御
        val->current.vel_error = val->tar.vel - val->current.vel;
        val->I.vel_error += val->current.vel_error / 1000.0;
        val->p.vel_error = (val->p.vel - val->current.vel) * 1000.0;

        control->V_l = val->current.vel_error * (control->v.Kp) + val->I.vel_error * (control->v.Ki) - val->p.vel_error * (control->v.Kd);
        control->V_r = val->current.vel_error * (control->v.Kp) + val->I.vel_error * (control->v.Ki) - val->p.vel_error * (control->v.Kd);

        // 角速度制御
        val->current.ang_error = val->tar.ang_vel - val->current.ang_vel;
        val->I.ang_error += val->current.ang_error / 1000.0;
        val->p.ang_error = (val->p.ang_vel - val->current.ang_vel) * 1000.0;

        control->V_l -= val->current.ang_error * (control->o.Kp) + val->I.ang_error * (control->o.Ki) - val->p.ang_error * (control->o.Kd);
        control->V_r += val->current.ang_error * (control->o.Kp) + val->I.ang_error * (control->o.Ki) - val->p.ang_error * (control->o.Kd);

        control->Duty_l = control->V_l / sens->BatteryVoltage;
        control->Duty_r = control->V_r / sens->BatteryVoltage;

        mot->setMotorSpeed(control->Duty_r, control->Duty_l, 0.0);
    }
    else
    {
        mot->setMotorSpeed(0.0, 0.0, 0.0);
    }

    // std::cout << "FB_ctl" << std::endl;
    return;
}

void Interrupt::calc_distance()
{ //  走行距離を計算する

    // エンコーダの値を取得
    sens->enc.data.l = -encL->readAngle();
    sens->enc.data.r = encR->readAngle();

    sens->enc.locate.l = sens->enc.data.l;
    sens->enc.locate.r = sens->enc.data.r;

    // 差分を計算
    sens->enc.diff_pulse.l = sens->enc.locate.l - sens->enc.p_locate.l;
    sens->enc.diff_pulse.r = sens->enc.locate.r - sens->enc.p_locate.r;

    // 制御１周期分前の値を保持
    sens->enc.p_locate.l = sens->enc.locate.l;
    sens->enc.p_locate.r = sens->enc.locate.r;

    if (sens->enc.diff_pulse.l > ENC_HALF)
        sens->enc.diff_pulse.l -= ENC_MAX - 1;
    if (sens->enc.diff_pulse.l < -ENC_HALF)
        sens->enc.diff_pulse.l += ENC_MAX - 1;
    if (sens->enc.diff_pulse.r > ENC_HALF)
        sens->enc.diff_pulse.r -= ENC_MAX - 1;
    if (sens->enc.diff_pulse.r < -ENC_HALF)
        sens->enc.diff_pulse.r += ENC_MAX - 1;

    float len_L = sens->enc.diff_pulse.l * MMPP;
    float len_R = sens->enc.diff_pulse.r * MMPP;

    // val->current.len += (len_L + len_R) / 2.0;

    // std::cout << "val->current.len : " << val->current.len * 1000.0 << std::endl;

    val->l.vel = len_L / 0.001; // 1ms
    val->r.vel = len_R / 0.001;

    // std::cout << "val->l.vel : " << val->l.vel * 1000.0 << std::endl;
    // std::cout << "val->r.vel : " << val->r.vel *1 000.0 << std::endl;

    if (!imu->in_survaeybias)
    { // サーベイバイアス中は加速度を計算しない
        _accel = (imu->accelY() * 9.80665) * 0.001;
    }

    float _vel = (val->l.vel + val->r.vel) / 2.0;
    // val->current.vel = _vel;
    val->current.vel = val->current.alpha * (val->current.vel + _accel) + (1.0 - val->current.alpha) * _vel;
    val->current.len += val->current.vel * 0.001;
    // std::cout << "val->current.vel : " << val->current.vel << std::endl;

    val->I.vel += val->current.vel; // 積分値更新

    val->p.vel = val->current.vel;

    // std::cout << "calc_dist" << std::endl;
    return;
}

void Interrupt::calc_angle()
{ //  角度を計算する
    // float _yaw = 0.0;
    if (!imu->in_survaeybias)
    { // サーベイバイアス中は角速度を計算しない
        sens->gyro.yaw = imu->gyroZ() - sens->gyro.ref;
    }

    val->current.ang_vel = sens->gyro.yaw * (M_PI / 180.0);

    val->current.rad += val->current.ang_vel / 1000.0;

    val->p.ang_vel = val->current.ang_vel;

    val->I.ang_vel += val->current.ang_vel; // 角速度積分値更新

    return;
}

void Interrupt::logging()
{
    esp_err_t err;

    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "logs");
    if (partition == NULL)
    {
        ESP_LOGE("logging", "partition not found");
        vTaskDelete(NULL);
    }

    err = esp_partition_erase_range(partition, 0, partition->size);
    if (err != ESP_OK)
    {
        ESP_LOGE("logging", "erase error");
        vTaskDelete(NULL);
    }
    uint32_t mem_offset = 0;
    int16_t adcs[10];

    ESP_LOGI("logging", "start logging");

    while (1)
    {
        //if (control->log_flag == TRUE)
        //{
            xSemaphoreTake(*on_logging, portMAX_DELAY); // セマフォが取得できるまで無制限に待機 （他タスクによって解放されるまでブロックされる）
            adcs[0] = sens->wall.val.fl;
            adcs[1] = sens->wall.val.l;
            adcs[2] = sens->wall.val.r;
            adcs[3] = sens->wall.val.fr;
            adcs[4] = (uint16_t)(sens->BatteryVoltage * 1000);
            adcs[5] = sens->wall.exist.fl;
            adcs[6] = sens->wall.exist.l;
            adcs[7] = sens->wall.exist.r;
            adcs[8] = sens->wall.exist.fr;
            adcs[9] = (uint16_t)(val->current.len * 1000);
            err = esp_partition_write(partition, mem_offset, adcs, sizeof(adcs));
            if (err != ESP_OK)
            {
                ESP_LOGE("logging", "write error");
                printf("%s\n", esp_err_to_name(err));

                break;
            }
            mem_offset += sizeof(adcs);
            if (mem_offset >= partition->size)
                break;

            
        //}
        //vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
    // std::cout << "logging" << std::endl;
    return;
}

void Interrupt::reset_I_gain()
{
    val->I.vel = 0.0;
    val->I.ang_vel = 0.0;
    val->I.wall_error = 0.0;
    return;
}

void Interrupt::interrupt()
{ //  xtaskcreate

    while (1)
    {
        calc_target();
        wall_control();
        feedback_control();
        calc_distance();
        calc_angle();

        if (control->log_flag == TRUE)
        {
            xSemaphoreGive(*on_logging);
        }
        
        control->time_count++;

        // printf("Duty_l : %f\n", control->Duty_l);
        // printf("Duty_r : %f\n", control->Duty_r);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
