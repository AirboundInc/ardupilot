#include "AC_PD.h"

const AP_Param::GroupInfo AC_PD::var_info[] = {
    // @Param: P
    // @DisplayName: PD Proportional Gain
    // @Description: PD controller proportional gain
    // @Range: 0.0 2.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("P",    0, AC_PD, _kp, 0.0f),

    // @Param: D
    // @DisplayName: PD Derivative Gain
    // @Description: PD controller derivative gain
    // @Range: 0.0 0.4
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("D",    1, AC_PD, _kd, 0.0f),

    AP_GROUPEND
};

// Constructor
AC_PD::AC_PD(float initial_p, float initial_d) :
    _kp(initial_p),
    _kd(initial_d)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// set_and_save_kp - set and save the proportional gain
void AC_PD::set_and_save_kp(float p)
{
    _kp.set_and_save(p);
}

// set_and_save_kd - set and save the derivative gain
void AC_PD::set_and_save_kd(float d)
{
    _kd.set_and_save(d);
}

// Update for one step
float AC_PD::update_pd(float error, float error_dot, float dt)
{
    // Calculate P and D terms
    float p_term = error * _kp;
    float d_term = error_dot * _kd;

    // Return combined PD term
    return p_term + d_term;
}
