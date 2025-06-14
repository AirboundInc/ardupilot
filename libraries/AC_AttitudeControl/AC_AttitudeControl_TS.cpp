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
    pitch_in = pitch_pid_out;
    if (enable_wind_comp) {
        pitch_in += _pitch_pid_boost_wind;
        pitch_in = constrain_float(pitch_in, -1.0, 1.0);
    }
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
    uint32_t now = AP_HAL::millis();

    if (now - last_log_ms >= LOGGING_INTERVAL_MS) {
        last_log_ms = now;
        log_write_ACTS0();
        log_write_ACTS1();
        log_write_ACTS2();
    }
}

void AC_AttitudeControl_TS::update_wind_boost()
{
    float theta = _ahrs.pitch_sensor * 0.01f; // centidegrees to degrees

    calculate_wind_force(theta);

    float wind_force_p = force_wind_perpendicular;
    float moment_wind = wind_force_p * (CG_CRAFT_M - CS_CRAFT_M);
    float phi_max_rad = DEG_TO_RAD * VECTORING_MAX_ANGLE_DEG;

    // Assuming hover thrust to be same as craft mass
    float thrust_hover = CRAFT_MASS_KG * GRAVITY_MSS;
    float thrust_p_max = thrust_hover * sinf(phi_max_rad);

    // -moment_wind / ((CG_CRAFT_M - MOTOR_POS_M) converts the countermoment demanded
    // to the perpendicular thrust required from the system.
    // In the conversion from perpendicular thrust to PID out, we need to multiply the term by 1/(thrust_p_max)

    float pitch_boost_wind = -moment_wind / ((CG_CRAFT_M - MOTOR_POS_M) * -thrust_p_max);

    static float last_boost = 0.0;

    _pitch_pid_boost_wind = (1 - BOOST_WEIGHT) * last_boost + BOOST_WEIGHT * pitch_boost_wind;
    last_boost = _pitch_pid_boost_wind;

    RC_Channel* rc = RC_Channels::rc_channel(BOOST_RC_CHANNEL - 1);
    if (rc == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unable to get output PWM for RC %f!", (float)BOOST_RC_CHANNEL);
    }
    enable_boost_pwm = rc->get_radio_in();

    if (enable_boost_pwm >= ENABLE_BOOST_THRESHOLD) {
        enable_wind_comp = true;
    } else {
        enable_wind_comp = false;
    }

    enable_wind_comp = enable_boost_pwm >= ENABLE_BOOST_THRESHOLD ? true : false;

    if (enable_wind_comp != prev_enable_wind_comp) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Setting enable wind comp to: %f", (float)enable_wind_comp);
    }
    prev_enable_wind_comp = enable_wind_comp;
}

// Calculate force of wind perpendicular to body while pitching
void AC_AttitudeControl_TS::calculate_wind_force(float pitch)
{
    // get current thrust vectoring angle for left motor
    uint16_t tv_pwm_left;
    bool success = SRV_Channels::get_output_pwm(SRV_Channel::k_tiltMotorLeft, tv_pwm_left);
    if (!success) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unable to get output PWM for tilt motor left");
        return;
    }

    uint16_t phi_min_pwm_left = get_servo_min(LEFT_TVSERVO_CHANNEL);
    uint16_t phi_max_pwm_left = get_servo_max(LEFT_TVSERVO_CHANNEL);
    phi_left = pwm_to_angle(tv_pwm_left, phi_min_pwm_left, phi_max_pwm_left);
    success = _telem.get_rpm(LEFT_ESC_INDEX, rpm_left);
    if (!success) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unable to get output RPM for %f", (float)LEFT_ESC_INDEX);
        rpm_left = 0.0f;
    }
    thrust_left = calculate_thrust(rpm_left, pitch, phi_left);

    // get current thrust vectoring angle for right motor
    uint16_t tv_pwm_right;
    success = SRV_Channels::get_output_pwm(SRV_Channel::k_tiltMotorRight, tv_pwm_right);
    if (!success) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unable to get output PWM for tilt motor right");
        return;
    }
    uint16_t phi_min_pwm_right = get_servo_min(RIGHT_TVSERVO_CHANNEL);
    uint16_t phi_max_pwm_right = get_servo_max(RIGHT_TVSERVO_CHANNEL);
    phi_right = pwm_to_angle(tv_pwm_right, phi_min_pwm_right, phi_max_pwm_right);
    success = _telem.get_rpm(RIGHT_ESC_INDEX, rpm_right);
    if (!success) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unable to get output RPM for %f", (float)RIGHT_ESC_INDEX);
        rpm_right = 0.0f;
    }
    thrust_right = calculate_thrust(rpm_right, pitch, phi_right);

    // NED frame accels
    // accel_ef = _ahrs.get_accel_ef();
    // accel_x_ef = accel_ef.x;
    // accel_y_ef = accel_ef.y;
    // accel_z_ef = accel_ef.z;

    accel_body = _ahrs.get_accel() - _ahrs.get_accel_bias();

    accel_x = accel_body.x;
    accel_y = accel_body.y;
    accel_z = accel_body.z;

    accel_zx.x = accel_z;
    accel_zx.y = accel_x;

    accel_elf_zx = _ahrs.body_to_earth2D_pitch(accel_zx);
    accel_z_elf = accel_elf_zx.x;
    accel_x_elf = accel_elf_zx.y;

    float total_thrust_horizontal = thrust_left.horizontal + thrust_right.horizontal;
    float total_thrust_vertical = thrust_left.vertical + thrust_right.vertical;

    force_net_z = accel_z_elf * CRAFT_MASS_KG;
    force_wind_z = force_net_z - total_thrust_horizontal;

    force_net_x = accel_x_elf * CRAFT_MASS_KG;
    force_wind_x = force_net_x - total_thrust_vertical;

    Vector2f force_wind_elf = { force_wind_z, force_wind_x };
    Vector2f force_wind_bf = _ahrs.earth_to_body2D_pitch(force_wind_elf);

    // local body frame z force of wind
    // force_wind_bf has components .x (Z) and .y (X), we want the Z one
    force_wind_perpendicular = force_wind_bf.x;
}

// Calculate thrust components using craft pitch and thrust vectoring angle
AC_AttitudeControl_TS::thrust_t AC_AttitudeControl_TS::calculate_thrust(float rpm, float pitch, float tv_angle)
{
    // TODO: Limit inputs to sane values/check

    thrust_t result;

    // Convention: Left is positive, counterclockwise is positive
    float theta_rad = pitch * DEG_TO_RAD;
    float phi_rad = tv_angle * DEG_TO_RAD;

    // Here we have to make phi_rad and theta_rad negative to
    // validate sign convention, as right is positive and CCW angles are positive
    float net_tilt_angle = -(theta_rad + phi_rad);

    const float current_throttle = _motors.get_throttle_out();
    float hover_throttle = _motors.get_throttle_hover();
    if (is_zero(hover_throttle)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Hover throttle zero! Current_throttle! (%f) Hover_throttle (%f)", current_throttle, hover_throttle);
        hover_throttle = DEFAULT_HOVER_THROTTLE;
    }

    static float throttle_to_thrust = (CRAFT_MASS_KG * GRAVITY_MSS) / hover_throttle;

    AP::logger().Write("THRT", "TimeUS,Throt,ThrotHov,ThrstThrotSlope,EnBst,EnWindC,EnThrotThst",
        "s-----", // seconds, unitless
        "F-----", // micro (1e-6), no mult (1e0)
        "Qfffff", // uint64_t, float
        AP_HAL::micros64(),
        current_throttle,
        hover_throttle,
        throttle_to_thrust,
        (float)enable_boost_pwm,
        (float)enable_wind_comp,
        (float)use_throttle_thrust_fallback);

    if (rpm <= MIN_RPM_FALLBACK_THRESHOLD && _motors.armed()) {
        // Fallback to throttle based thrust estimation if rpm is zero
        result.thrust = current_throttle * throttle_to_thrust + THROTTLE_THRUST_INTERCEPT; // Newtons
        calculate_thrust_components(result, net_tilt_angle, phi_rad);
        use_throttle_thrust_fallback = true;

        return result;
    }

    use_throttle_thrust_fallback = false;

#ifdef SITL_DEBUG
    // https://www.rcgroups.com/forums/showthread.php?288091-How-to-calculate-thrust-given-RPM-prop-pitch-and-prop-diameter
    // Using formula for propellers Pitch/Diameter <=0.6
    // Thrust = P X D^3 X RPM^2 X 10^-10 oz.
    float prop_pitch = PROPELLER_PITCH_IN;
    float prop_dia = PROPELLER_DIAMETER_IN;
    // convert from oz to kg
    double conversion_const = 1e-10 / 35.274;
    double compensation_const = 0.8626;

    result.thrust = prop_pitch * prop_dia * prop_dia * prop_dia * rpm * rpm * conversion_const * compensation_const;

#else
    // generated via experimental mapping of rpm to thrust
    double coeff_a = 6.369e-8;
    double coeff_b = -2.724e-5;
    double coeff_c = 0.007676;
    result.thrust = coeff_a * rpm * rpm + coeff_b * rpm + coeff_c;
#endif

    // convert from kg to N
    result.thrust *= GRAVITY_MSS;

    calculate_thrust_components(result, net_tilt_angle, phi_rad);

    // update fallback conversion factor in normal case
    if (is_positive(current_throttle)) {
        throttle_to_thrust = (result.thrust - THROTTLE_THRUST_INTERCEPT) / current_throttle;
    }

    return result;
}

// calculate horizontal, vertical, perpendicular, parallel components of thrust
void AC_AttitudeControl_TS::calculate_thrust_components(thrust_t& result, float tilt_angle_rad, float vectoring_angle_rad)
{
    float net_tilt_angle = tilt_angle_rad;
    float phi_rad = vectoring_angle_rad;

    result.horizontal = result.thrust * sinf(net_tilt_angle);
    result.vertical = result.thrust * cosf(net_tilt_angle);
    result.perpendicular = result.thrust * sinf(-phi_rad);
    result.parallel = result.thrust * cosf(-phi_rad);
}

// Convert pwm to thrust vectoring angle in degrees
float AC_AttitudeControl_TS::pwm_to_angle(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max)
{
    // TODO: If pwm < pwm_min - delta, set pwm to pwm_min and also for max
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

    tv_angle *= -1.0f;
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

void AC_AttitudeControl_TS::log_write_ACTS0()
{
    const struct log_ACTS0 pkt {
        LOG_PACKET_HEADER_INIT(LOG_ACTS0_MSG),
        time_us : AP_HAL::micros64(),
        force_net_z : force_net_z,
        force_net_x : force_net_x,
        force_wind_p : force_wind_perpendicular,
        phi_left : phi_left,
        phi_right : phi_right,
        rpm_left : rpm_left,
        rpm_right : rpm_right,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AC_AttitudeControl_TS::log_write_ACTS1()
{
    const struct log_ACTS1 pkt2 {
        LOG_PACKET_HEADER_INIT(LOG_ACTS1_MSG),
        time_us : AP_HAL::micros64(),
        thrust_left : thrust_left.thrust,
        thrust_left_h : thrust_left.horizontal,
        thrust_left_v : thrust_left.vertical,
        thrust_left_p : thrust_left.perpendicular,
        thrust_right : thrust_right.thrust,
        thrust_right_h : thrust_right.horizontal,
        thrust_right_v : thrust_right.vertical,
        thrust_right_p : thrust_right.perpendicular,
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void AC_AttitudeControl_TS::log_write_ACTS2()
{
    const struct log_ACTS2 pkt3 {
        LOG_PACKET_HEADER_INIT(LOG_ACTS2_MSG),
        time_us : AP_HAL::micros64(),
        body_acc_x : accel_x,
        body_acc_y : accel_y,
        body_acc_z : accel_z,
        elf_acc_x : accel_x_elf,
        elf_acc_z : accel_z_elf,
        pitch_pid_boost_wind : _pitch_pid_boost_wind,
        pid_out_boosted : pitch_in,
    };
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}
