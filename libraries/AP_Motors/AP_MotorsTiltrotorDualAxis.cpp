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
 */

/*
 *       AP_MotorsTiltrotorDualAxis.cpp - motor mixer for dual-axis (V22-style) tiltrotors
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTiltrotorDualAxis.h"
#include <SRV_Channel/SRV_Channel.h>

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTiltrotorDualAxis::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight) || SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);

    // Axis 1 (transition tilt, k_tiltMotorLeft/Right) is driven directly by
    // Tiltrotor::dual_axis_output() in ArduPlane, not by this mixer, so no
    // default channel/range is set up for it here.

    // right Axis 2 vectoring servo defaults to servo output 5
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRightVec, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRightVec, SERVO_OUTPUT_RANGE);

    // left Axis 2 vectoring servo defaults to servo output 6
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeftVec, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeftVec, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_VTOL_TILTROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
}

// calculate outputs to the motors
// TODO: replace with the dual-axis-specific mixing strategy. For now this
// mirrors AP_MotorsTailsitter::output_armed_stabilizing() as a placeholder
// so the class remains fully functional while the real strategy is written.
void AP_MotorsTiltrotorDualAxis::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;                 // highest motor value
    float   thrust_min;                 // lowest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = thr_lin.get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;
    const float max_boost_throttle = _throttle_avg_max * compensation_gain;

    // never boost above max, derived from throttle mix params
    const float min_throttle_out = MIN(_external_min_throttle, max_boost_throttle);
    const float max_throttle_out = _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above min and below current limited throttle
    if (throttle_thrust <= min_throttle_out) {
        throttle_thrust = min_throttle_out;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= max_throttle_out) {
        throttle_thrust = max_throttle_out;
        limit.throttle_upper = true;
    }

    if (roll_thrust >= 1.0) {
        // cannot split motor outputs by more than 1
        roll_thrust = 1;
        limit.roll = true;
    }
    // Global mixer for the dual-axis tiltrotor with roll yaw swaping based on the elbow tilt angle.
    const float inverse_term = 1.0f / MAX(FLT_EPSILON, cosf(2.0f*_elbow_tilt_angle));
    const float differential_thrust = (roll_thrust *cosf(_elbow_tilt_angle) + yaw_thrust * -sinf(_elbow_tilt_angle)) * inverse_term;
    const float differential_TV = (roll_thrust * -sinf(_elbow_tilt_angle) + yaw_thrust * cosf(_elbow_tilt_angle)) * inverse_term;

    // calculate left and right throttle outputs
    _thrust_left  = throttle_thrust + differential_thrust * 0.5f;
    _thrust_right = throttle_thrust - differential_thrust * 0.5f;

    thrust_max = MAX(_thrust_right,_thrust_left);
    thrust_min = MIN(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) {
        // if max thrust is more than one reduce average throttle
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
    } else if (thrust_min < 0.0) {
        // if min thrust is less than 0 increase average throttle
        // but never above max boost
        thr_adj = -thrust_min;
        if ((throttle_thrust + thr_adj) > max_boost_throttle) {
            thr_adj = MAX(max_boost_throttle - throttle_thrust, 0.0);
            // in this case we throw away some roll output, it will be uneven
            // constraining the lower motor more than the upper
            // this unbalances torque, but motor torque should have significantly less control power than tilts / control surfaces
            // so its worth keeping the higher roll control power at a minor cost to yaw
            limit.roll = true;
        }
        limit.throttle_lower = true;
    }

    // Add adjustment to reduce average throttle
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);

    _throttle = throttle_thrust;

    // compensation_gain can never be zero
    // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
    if (_has_diff_thrust) {
        _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
    } else {
        _throttle_out = throttle_thrust / compensation_gain;
    }

    // thrust vectoring
    _tilt_left  = pitch_thrust - differential_TV;
    _tilt_right = pitch_thrust + differential_TV;
}

void AP_MotorsTiltrotorDualAxis::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _actuator[2] = 0.0f;
            _external_min_throttle = 0.0;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            _external_min_throttle = 0.0;
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thr_lin.thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[1], thr_lin.thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thr_lin.thrust_to_actuator(_throttle));
            break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[1]));

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    // pitch/yaw vectoring output goes to the independent Axis 2 servos
    // instead of AP_MotorsTailsitter's own tilt servos (k_tiltMotorLeft/Right)
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeftVec, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRightVec, _tilt_right*SERVO_OUTPUT_RANGE);
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTiltrotorDualAxis::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right Axis 2 vectoring servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRightVec, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left Axis 2 vectoring servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeftVec, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
