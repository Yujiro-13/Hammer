#ifndef UI_HPP
#define UI_HPP

#include "../Micromouse.hpp"

#define _interface struct

_interface UI : Micromouse
{
    virtual void main_task() = 0;
};

#endif // UI_HPP