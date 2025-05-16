#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Logger/AP_Logger.h>

// TODO: Turn into params
const float CRAFT_MASS_KG = 1.3;
const float CG_CRAFT_M = 0.225; // Center of Gravity of the drone in m. Measured as distance to nosetip along chord axis
const float CS_CRAFT_M = 0.250; // Center of Surface area of the drone in m. Where the point vector of wind acts on. Measured from distance to nosetip along chord axis.
const float MOTOR_POS_M = 0.100; // Position of the motor in m. Measured from distance to nosetip along chord axis with motor at neutral position (Phi=0)
const float VECTORING_MIN_ANGLE_DEG = -45.0;
const float VECTORING_MAX_ANGLE_DEG = 45.0;

const uint8_t LEFT_SERVO_CHANNEL = 14, RIGHT_SERVO_CHANNEL = 4;

const uint32_t LOGGING_INTERVAL_MS = 40; // 25Hz

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

private:
    AP_ESC_Telem& _telem = AP::esc_telem();
    float _pitch_pid_boost_wind = 0.0;
    uint32_t last_log_ms = 0;

    struct thrust_t {
        float thrust;
        float horizontal;
        float vertical;
        float perpendicular;
    };

    thrust_t thrust_left, thrust_right;

    // wind force boost methods
    void update_wind_boost();
    float calculate_wind_force(float pitch);
    thrust_t calculate_thrust(float rpm, float pitch, float tv_angle);

    // Helper methods
    uint16_t get_servo_min(uint8_t channel);
    uint16_t get_servo_max(uint8_t channel);
    float get_param_value_by_name(char *param_name, float default_value);
    float pwm_to_angle(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max);

    void write_AttControlTS_log(float accel_z);
};
