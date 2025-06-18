/*
  Generic PD algorithm for controllers
*/

#include <AP_Math/AP_Math.h>
#include "AC_PD.h"

const AP_Param::GroupInfo AC_PD::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    1, AC_PD, _kp, default_kp),

    // @Param: D
    // @DisplayName: PID Differential Gain
    // @Description: D Gain which produces an output that is proportional to the rate of the error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D",    2, AC_PD, _kd, default_kd),

    AP_GROUPEND
};

// Constructor
AC_PD::AC_PD(float initial_p, float initial_d) :
    default_kp(initial_p),
    default_kd(initial_d)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
}

float AC_PD::update(float error, float dt)
{
    const float err = error;
    // initialize with error for first loop
    static float err_last = err;

    // TODO: Add moving avg filter similar to AC_PID
    if (!is_zero(dt)){
        derivative = (err - err_last) / dt;
    }
    output_D = _kd * derivative;
    output_P = _kp * err;

    err_last = err;
    log_Write_PD_details(error);

    return output_P + output_D;
}

void AC_PD::load_gains()
{
    _kp.load();
    _kd.load();
}

void AC_PD::save_gains()
{
    _kp.save();
    _kd.save();
}


void AC_PD::log_Write_PD_details(float error)
{
    // @LoggerMessage: ATPD
    // @Description: Attitude angle controller PD loop data packet
    // @Vehicles: Copter
    // @Field: TimeUS: Time since system startup
    // @Field: Error: Error in angle
    // @Field: Derivative: Derivative of error
    // @Field: P: Proportional term output
    // @Field: D: Derivative term output
    // @Field: Output: Total output

    AP::logger().WriteStreaming("ATPD", "TimeUS,Error,Derivative,P,D,Output",
        "sd----", // seconds, degrees
        "F00000", // micro (1e-6), no mult (1e0)
        "Qfffff", // uint64_t, float
        AP_HAL::micros64(),
        error, derivative, output_P, output_D, output_P+output_D);
}