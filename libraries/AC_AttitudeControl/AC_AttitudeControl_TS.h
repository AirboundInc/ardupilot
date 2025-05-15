#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"

// TODO: Turn into params
const float CRAFT_MASS_KG = 1.3;
const float VECTORING_MIN_ANGLE_DEG = -45.0;
const float VECTORING_MAX_ANGLE_DEG = 45.0;

class AC_AttitudeControl_TS : public AC_AttitudeControl_Multi
{
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_TS() {}

    // Ensure attitude controllers have zero errors to relax rate controller output
    // Relax only the roll and yaw rate controllers if exclude_pitch is true
    virtual void relax_attitude_controllers(bool exclude_pitch) override;
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds) override;

    // run custom rate loop for tailsitters
    virtual void rate_controller_run() override;

    // wind force boost methods
    float calculate_thrust(float rpm, float pitch, float tv_angle);
    float calculate_wind_force(float pitch);

    // Helper methods
    uint16_t get_servo_min(uint8_t channel);
    float get_param_value_by_name(char *param_name, float default_value);
    float pwm_to_angle(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max);
};
