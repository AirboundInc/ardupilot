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
    // // boost angle_p/pd each cycle on high throttle slew
    // update_throttle_gain_boost();

    // // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    // update_throttle_rpy_mix();

    // _ang_vel_body += _sysid_ang_vel_body;

    // Vector3f gyro_latest = _ahrs.get_gyro_latest();

    // _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x,  _dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
    // _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    // _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y,  _dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y);
    // _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    // _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z,  _dt, _motors.limit.yaw, _pd_scale.z) + _actuator_sysid.z);
    // _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    // _sysid_ang_vel_body.zero();
    // _actuator_sysid.zero();

    // _pd_scale_used = _pd_scale;
    // _pd_scale = VECTORF_111;

    // control_monitor_update();
    // gcs().send_text(MAV_SEVERITY_INFO, "Running custom rate loop for tailsitters!!");
    get_servo_min(4);
}

float AC_AttitudeControl_TS::calculate_wind_force(float pitch)
{
    float force_wind = 0.0; // Newtons

    return force_wind;
}

// Calculate thrust components using craft pitch and thrust vectoring angle
float AC_AttitudeControl_TS::calculate_thrust(float rpm, float pitch, float tv_angle)
{
    // float theta_rad = pitch * DEG_TO_RAD;
    // float phi_rad = tv_angle * DEG_TO_RAD;

    return 0.0;
}
float AC_AttitudeControl_TS::pwm_to_angle(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max)
{
    return 0.0;
}

uint16_t AC_AttitudeControl_TS::get_servo_min(uint8_t channel)
{
    char param_name[17];
    hal.util->snprintf(param_name, sizeof(param_name), "SERVO%u_MIN", channel);

    float servo_min = get_param_value_by_name(param_name, 1000);

    return (uint16_t)servo_min;
}

float AC_AttitudeControl_TS::get_param_value_by_name(char* param_name, float default_value)
{
    float value;

    if (!AP_Param::get(param_name, value)) {
        // Return default if parameter not found
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Param read failed (%s)", param_name);
        value = default_value;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>>Got min servo value for (%s)", param_name);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Value: (%f)", (float)value);

    return value;
}
