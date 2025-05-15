/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   This class inherits from AC_AttitudeControl_Multi and provides functionality
   specific to tailsitter quadplanes.
   1) "body-frame" roll control mode for all tailsitters
   2) a relax_attitude_controller method needed for coping with vectored tailsitters
 */
#include "AC_AttitudeControl_TS.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AC_AttitudeControl_TS::relax_attitude_controllers(bool exclude_pitch)
{
    // If exclude_pitch: relax roll and yaw rate controller outputs only,
    // leaving pitch controller active to let TVBS motors tilt up while in throttle_wait
    if (exclude_pitch) {
        // Get the current attitude quaternion
        Quaternion current_attitude;
        _ahrs.get_quat_body_to_ned(current_attitude);

        Vector3f current_eulers;
        current_attitude.to_euler(current_eulers.x, current_eulers.y, current_eulers.z);

        // set target attitude to zero pitch with (approximate) current roll and yaw
        // by rotating the current_attitude quaternion by the error in desired pitch
        Quaternion pitch_rotation;
        pitch_rotation.from_axis_angle(Vector3f(0, -1, 0), current_eulers.y);
        _attitude_target = current_attitude * pitch_rotation;
        _attitude_target.normalize();
        _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
        _attitude_ang_error = current_attitude.inverse() * _attitude_target;

        // Initialize the roll and yaw angular rate variables to the current rate
        _ang_vel_target = _ahrs.get_gyro();
        ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
        _ang_vel_body.x = _ahrs.get_gyro().x;
        _ang_vel_body.z = _ahrs.get_gyro().z;

        // Reset the roll and yaw I terms
        get_rate_roll_pid().reset_I();
        get_rate_yaw_pid().reset_I();
    } else {
        // relax all attitude controllers
        AC_AttitudeControl::relax_attitude_controllers();
    }
}

// Command euler yaw rate and pitch angle with roll angle specified in body frame
// (used only by tailsitter quadplanes)
// If plane_controls is true, swap the effects of roll and yaw as euler pitch approaches 90 degrees
void AC_AttitudeControl_TS::input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_yaw_rate = radians(euler_yaw_rate_cds*0.01f);
    float euler_pitch    = radians(constrain_float(euler_pitch_cd * 0.01f, -90.0f, 90.0f));
    float body_roll      = radians(-body_roll_cd * 0.01f);

    const float cpitch = cosf(euler_pitch);
    const float spitch = fabsf(sinf(euler_pitch));

    // Compute attitude error
    Quaternion attitude_body;
    Quaternion error_quat;
    _ahrs.get_quat_body_to_ned(attitude_body);
    error_quat = attitude_body.inverse() * _attitude_target;
    Vector3f att_error;
    error_quat.to_axis_angle(att_error);

    // update heading
    float yaw_rate = euler_yaw_rate;
    if (plane_controls) {
        yaw_rate = (euler_yaw_rate * spitch) + (body_roll * cpitch);
    }
    // limit yaw error
    float yaw_error = fabsf(att_error.z);
    float error_ratio = yaw_error / M_PI_2;
    if (error_ratio > 1) {
        yaw_rate /= (error_ratio * error_ratio);
    }
    _euler_angle_target.z = wrap_PI(_euler_angle_target.z + yaw_rate * _dt);

    // init attitude target to desired euler yaw and pitch with zero roll
    _attitude_target.from_euler(0, euler_pitch, _euler_angle_target.z);

    // apply body-frame yaw/roll (this is roll/yaw for a tailsitter in forward flight)
    // rotate body_roll axis by |sin(pitch angle)|
    Quaternion bf_roll_Q;
    bf_roll_Q.from_axis_angle(Vector3f(0, 0, spitch * body_roll));

    // rotate body_yaw axis by cos(pitch angle)
    Quaternion bf_yaw_Q;
    if (plane_controls) {
        bf_yaw_Q.from_axis_angle(Vector3f(cpitch, 0, 0), euler_yaw_rate);
    } else {
        bf_yaw_Q.from_axis_angle(Vector3f(-cpitch * body_roll, 0, 0));
    }
    _attitude_target = _attitude_target * bf_roll_Q * bf_yaw_Q;

    // _euler_angle_target roll and pitch: Note: roll/yaw will be indeterminate when pitch is near +/-90
    // These should be used only for logging target eulers, with the caveat noted above.
    // Also note that _attitude_target.from_euler() should only be used in special circumstances
    // such as when attitude is specified directly in terms of Euler angles.
    //    _euler_angle_target.x = _attitude_target.get_euler_roll();
    //    _euler_angle_target.y = euler_pitch;

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();

    // Compute attitude error
    error_quat = attitude_body.inverse() * _attitude_target;
    error_quat.to_axis_angle(att_error);

    // Compute the angular velocity target from the attitude error
    _ang_vel_body = update_ang_vel_target_from_att_error(att_error);
}

void AC_AttitudeControl_TS::rate_controller_run() {
    // boost angle_p/pd each cycle on high throttle slew
    update_throttle_gain_boost();

    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    _ang_vel_body += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x, _dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    // add custom wind force based boost to pitch pid output
    float pitch_pid_out = get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, _dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y;
    update_wind_boost();
    float pitch_in = pitch_pid_out + _pitch_pid_boost_wind;
    pitch_in = constrain_float(pitch_in, -1.0, 1.0);
    _motors.set_pitch(pitch_in);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, _dt, _motors.limit.yaw, _pd_scale.z) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff() * _feedforward_scalar);

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    _pd_scale_used = _pd_scale;
    _pd_scale = VECTORF_111;

    control_monitor_update();

    // Log/Run stuff at 1Hz
    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();

    if (now - last_log_ms >= 1000) {
        last_log_ms = now;
        get_servo_min(4);
    }
}

void AC_AttitudeControl_TS::update_wind_boost()
{
    float theta = _ahrs.pitch_sensor * 0.01f; // centidegrees to degrees
    float wind_force_p = calculate_wind_force(theta);

    float moment_wind = wind_force_p * (CS_CRAFT_M - CG_CRAFT_M);
    float phi_max_rad = DEG_TO_RAD * VECTORING_MAX_ANGLE_DEG;

    // Assuming hover thrust to be same as craft mass
    float thrust_hover = CRAFT_MASS_KG * GRAVITY_MSS;
    float thrust_p_max = thrust_hover * sinf(phi_max_rad);

    // -moment_wind / ((CG_CRAFT_M - MOTOR_POS_M) converts the countermoment demanded
    // to the perpendicular thrust required from the system.
    // In the conversion from perpendicular thrust to PID out, we need to multiply the term by 1/(thrust_p_max)

    float pitch_boost_wind = -moment_wind / ((CG_CRAFT_M - MOTOR_POS_M) * thrust_p_max);

    static float last_boost = 0.0;

    _pitch_pid_boost_wind = 0.95 * last_boost + 0.05 * pitch_boost_wind;
    last_boost = _pitch_pid_boost_wind;
}

// Calculate force of wind perpendicular to body while pitching
float AC_AttitudeControl_TS::calculate_wind_force(float pitch)
{
    // get current thrust vectoring angle for left motor
    uint16_t tv_pwm_left;
    SRV_Channels::get_output_pwm(SRV_Channel::k_tiltMotorLeft, tv_pwm_left);
    uint16_t phi_min_pwm_left = get_servo_min(LEFT_SERVO_CHANNEL);
    uint16_t phi_max_pwm_left = get_servo_max(LEFT_SERVO_CHANNEL);
    float phi_left = pwm_to_angle(tv_pwm_left, phi_min_pwm_left, phi_max_pwm_left);
    float rpm_left = _telem.get_average_motor_rpm(SRV_Channel::k_throttleLeft);
    float thrust_p_left = calculate_thrust(rpm_left, pitch, phi_left);

    // get current thrust vectoring angle for right motor
    uint16_t tv_pwm_right;
    SRV_Channels::get_output_pwm(SRV_Channel::k_tiltMotorRight, tv_pwm_right);
    uint16_t phi_min_pwm_right = get_servo_min(RIGHT_SERVO_CHANNEL);
    uint16_t phi_max_pwm_right = get_servo_max(RIGHT_SERVO_CHANNEL);
    float phi_right = pwm_to_angle(tv_pwm_right, phi_min_pwm_right, phi_max_pwm_right);
    float rpm_right = _telem.get_average_motor_rpm(SRV_Channel::k_throttleRight);
    float thrust_p_right = calculate_thrust(rpm_right, pitch, phi_right);

    float total_thrust_perpendicular = thrust_p_left + thrust_p_right;

    float accel_z = _ahrs.get_accel_ef().z;
    float force_net_perpendicular = accel_z * CRAFT_MASS_KG;
    float force_wind_perpendicular = force_net_perpendicular - total_thrust_perpendicular; // Newtons

    return force_wind_perpendicular;
}

// Calculate thrust components using craft pitch and thrust vectoring angle
float AC_AttitudeControl_TS::calculate_thrust(float rpm, float pitch, float tv_angle)
{
    // TODO: Limit inputs to sane values/check

    float theta_rad = pitch * DEG_TO_RAD;
    float phi_rad = tv_angle * DEG_TO_RAD;

    float thrust; // thrust from motor
    float thrust_h; // horizontal component in global frame
    float thrust_v; // vertical component in global frame
    float thrust_p; // perpendicular component to body

    // generated via experimental mapping of rpm to thrust
    double coeff_a = 6.369 * 10e-8;
    double coeff_b = -2.724 * 10e-5;
    double coeff_c = 0.007676;

    thrust = coeff_a * rpm * rpm + coeff_b * rpm + coeff_c;
    thrust *= GRAVITY_MSS;

    thrust_h = thrust * sinf(theta_rad + phi_rad);
    thrust_v = thrust * cosf(theta_rad + phi_rad);
    thrust_p = thrust * sinf(phi_rad);

    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();

    if (now - last_log_ms >= 1000) {
        last_log_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>>Thrust values (h,v,p): %f %f %f", thrust_h, thrust_v, thrust_p);
    }

    return thrust_p;
}

// Convert pwm to thrust vectoring angle in degrees
float AC_AttitudeControl_TS::pwm_to_angle(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max)
{
    uint16_t error_delta = 250;
    // Check if the PWM value is within the valid range
    if (pwm < pwm_min - error_delta || pwm > pwm_max + error_delta) {
        // Handle invalid input
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PWM value out of range! (%f)", (float)pwm);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PWM min, max! (%f, %f)", (float)pwm_min, (float)pwm_max);
        return 0.0f; // Return a default value or handle error as needed
    }

    // TODO: Update to analytical curve based mapping once not for SITL
    float tv_angle = (pwm - pwm_min) * (VECTORING_MAX_ANGLE_DEG - VECTORING_MIN_ANGLE_DEG) / (pwm_max - pwm_min) + VECTORING_MIN_ANGLE_DEG;

    return tv_angle;
}

uint16_t AC_AttitudeControl_TS::get_servo_min(uint8_t channel)
{
    char param_name[17];
    hal.util->snprintf(param_name, sizeof(param_name), "SERVO%u_MIN", channel);

    float servo_min = get_param_value_by_name(param_name, 845);

    return (uint16_t)servo_min;
}

uint16_t AC_AttitudeControl_TS::get_servo_max(uint8_t channel)
{
    char param_name[17];
    hal.util->snprintf(param_name, sizeof(param_name), "SERVO%u_MAX", channel);

    float servo_max = get_param_value_by_name(param_name, 2145);

    return (uint16_t)servo_max;
}

float AC_AttitudeControl_TS::get_param_value_by_name(char* param_name, float default_value)
{
    float value;

    if (!AP_Param::get(param_name, value)) {
        // Return default if parameter not found
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Param read failed (%s)", param_name);
        value = default_value;
    }

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>>Got value for %s: %f", param_name, (float)value);
    return value;
}
