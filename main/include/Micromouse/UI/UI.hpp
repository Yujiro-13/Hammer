#ifndef UI_HPP
#define UI_HPP

#include "../Micromouse.hpp"
#include "../Motion/Adachi.hpp"
#include "include/files.hpp"
#include "../../Driver/adc.hpp"
#include "../../Driver/AS5047P.hpp"
#include "../../Driver/Buzzer.hpp"
#include "../../Driver/MPU6500.hpp"
#include "../../Driver/PCA9632.hpp"
#include "../../Driver/Motor.hpp"
#include "../../Driver/NeoPixel.hpp"

#define _interface struct

_interface UI : Micromouse
{
    virtual void main_task() = 0;
    virtual void ref_by_motion(Adachi &_adachi) = 0;
    virtual void set_device(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) = 0;
};

#endif // UI_HPP