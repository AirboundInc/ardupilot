#pragma once

/// @file    AC_PD.h
/// @brief   Proportional Derivative (PD) controller

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <stdlib.h>
#include <cmath>

/// @class   AC_PD
/// @brief   Class for PD control
class AC_PD {
public:
    // Constructor
    AC_PD(float initial_p, float initial_d);

    // set_and_save_kp - set and save the proportional gain
    void set_and_save_kp(float p);

    // set_and_save_kd - set and save the derivative gain 
    void set_and_save_kd(float d);

    // kp - return proportional gain
    float kP() const { return _kp.get(); }

    // kd - return derivative gain
    float kD() const { return _kd.get(); }

    // get gains
    const Vector2f& get_gains() const { return Vector2f(_kp.get(), _kd.get()); }

    // Return PD term given error and delta time
    float update_pd(float error, float error_dot);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Parameters
    AP_Float _kp;     // proportional gain
    AP_Float _kd;     // derivative gain
};
