#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Logger/AP_Logger.h>

#define SITL_DEBUG // Comment out if not doing SITL DEBUG

// TODO: Turn into params
const float CRAFT_MASS_KG = 2.5;
const float PROPELLER_PITCH_IN = 10;
const float PROPELLER_DIAMETER_IN = 15;

// CG_CRAFT_M
// Center of Gravity of the drone in m.
// Measured as distance to nosetip along chord axis

// CS_CRAFT_M
// Center of Surface area of the drone in m.
// Where the point vector of wind acts on.
// Measured from distance to nosetip along chord axis.

const float MOTOR_POS_M = 0.100; // Position of the motor in m. Measured from distance to nosetip along chord axis with motor at neutral position (Phi=0)
const float VECTORING_MIN_ANGLE_DEG = -45.0;
const float VECTORING_MAX_ANGLE_DEG = 45.0;

#ifdef SITL_DEBUG
// SITL configuration
const uint8_t LEFT_TVSERVO_CHANNEL = 3;
const uint8_t RIGHT_TVSERVO_CHANNEL = 4;
const uint8_t LEFT_ESC_INDEX = 1;
const uint8_t RIGHT_ESC_INDEX = 1;

const float CG_CRAFT_M = 0.230;
const float CS_CRAFT_M = 0.230;
// const float CS_CRAFT_M = 0.336;
#else
// TRT (hardware) configuration
const uint8_t LEFT_TVSERVO_CHANNEL = 14;
const uint8_t RIGHT_TVSERVO_CHANNEL = 4;
const uint8_t LEFT_ESC_INDEX = 1;
const uint8_t RIGHT_ESC_INDEX = 2;

const float CG_CRAFT_M = 0.230;
const float CS_CRAFT_M = 0.336;
#endif

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
    struct thrust_t {
        float thrust;
        float horizontal;
        float vertical;
        float perpendicular;
    };

    // wind force boost methods
    void update_wind_boost();
    float calculate_wind_force(float pitch);
    thrust_t calculate_thrust(float rpm, float pitch, float tv_angle);

    // Helper methods
    uint16_t get_servo_min(uint8_t channel);
    uint16_t get_servo_max(uint8_t channel);
    float get_param_value_by_name(char *param_name, float default_value);
    float pwm_to_angle(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max);

    void log_write_ACTS0();
    void log_write_ACTS1();
    void log_write_ACTS2();

    AP_ESC_Telem& _telem = AP::esc_telem();
    uint32_t last_log_ms = 0;

    float _pitch_pid_boost_wind = 0.0;

    thrust_t thrust_left, thrust_right;
    float accel_z, accel_y, accel_x, accel_z_g_comp;
    Vector3f accel_body;
    float force_net_perpendicular, force_wind_perpendicular;
    float phi_left, phi_right; // thrust vectoring angles of each tilt servo
    float rpm_left, rpm_right;
};
