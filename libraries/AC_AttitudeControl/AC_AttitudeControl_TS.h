#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Logger/AP_Logger.h>

#define SITL_DEBUG // Comment out if not doing SITL DEBUG

const bool ENABLE_WIND_COMP = true;
// const bool ENABLE_WIND_COMP = false;

// TODO: Turn into params
const float CRAFT_MASS_KG = 2.5;
const float PROPELLER_PITCH_IN = 10;
const float PROPELLER_DIAMETER_IN = 15;
const float BOOST_WEIGHT = 0.05; // Use 5 percent of new value

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

// const uint8_t LEFT_ESC_INDEX = 1;
// const uint8_t RIGHT_ESC_INDEX = 1;
// TODO: revert after throttle fallback test, 0 is wrong, 1 is correct for SITL ESC channel

const uint8_t LEFT_ESC_INDEX = 0;
const uint8_t RIGHT_ESC_INDEX = 0;

const float CG_CRAFT_M = 0.230;
const float CS_CRAFT_M = 0.336;
const float THROTTLE_THRUST_INTERCEPT = 0.89; // analytically calculated
#else
// TRT (hardware) configuration
const uint8_t LEFT_TVSERVO_CHANNEL = 14;
const uint8_t RIGHT_TVSERVO_CHANNEL = 4;
const uint8_t LEFT_ESC_INDEX = 11; // l_throttle - channel 12
const uint8_t RIGHT_ESC_INDEX = 10; // r_throttle - channel 11

const float CG_CRAFT_M = 0.230;
const float CS_CRAFT_M = 0.318;
const float THROTTLE_THRUST_INTERCEPT = 0.89; // TODO: Get this value!
#endif

const float DEFAULT_HOVER_THROTTLE = 0.3;

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
        float horizontal; // global frame
        float vertical; // global frame
        float perpendicular; // to local body surface
        float parallel; // to local body surface
    };

    // wind force boost methods
    void update_wind_boost();
    void calculate_wind_force(float pitch);
    thrust_t calculate_thrust(float rpm, float pitch, float tv_angle);
    void calculate_thrust_components(thrust_t& result, float tilt_angle_rad, float vectoring_angle_rad);

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

    float _pitch_pid_boost_wind = 0.0, pitch_in = 0.0;

    thrust_t thrust_left, thrust_right;
    // body frame accels
    float accel_z, accel_y, accel_x;
    // earth frame (NED) accels
    float accel_x_ef, accel_y_ef, accel_z_ef;
    // earth local 2D frame ZX plane accels
    float accel_x_elf, accel_z_elf;

    Vector3f accel_body, accel_ef;
    Vector2f accel_elf_zx, accel_zx;

    float force_net_z, force_net_x;
    float force_wind_z, force_wind_x;
    float force_wind_perpendicular;

    float phi_left, phi_right; // thrust vectoring angles of each tilt servo
    float rpm_left, rpm_right;
};
