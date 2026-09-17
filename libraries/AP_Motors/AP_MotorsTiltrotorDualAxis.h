/// @file	AP_MotorsTiltrotorDualAxis.h
/// @brief	Motor control class for dual-axis (V22-style) tiltrotors
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"

/// @class      AP_MotorsTiltrotorDualAxis
/// Mixes roll/pitch/yaw/throttle into left/right motor thrust and Axis 2
/// vectoring demand with its own dual-axis-specific strategy (see
/// output_armed_stabilizing() in the .cpp). The computed pitch/yaw
/// vectoring output is sent to the Axis 2 attitude-vectoring servos
/// (k_tiltMotorLeftVec/RightVec) instead of AP_MotorsTailsitter's own tilt
/// servos (k_tiltMotorLeft/Right). Those Axis 1 channels are reserved for
/// the transition tilt angle, driven separately by
/// Tiltrotor::dual_axis_output() in ArduPlane.
class AP_MotorsTiltrotorDualAxis : public AP_MotorsTailsitter {
public:

    using AP_MotorsTailsitter::AP_MotorsTailsitter;

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set the current Axis 1 (elbow) tilt position, as passed by
    // Tiltrotor::dual_axis_output() (ArduPlane): this is axis1_pos,
    // -(current_tilt * SERVO_MAX), NOT degrees
    float set_elbow_tilt_angle(float angle) { return _elbow_tilt_angle = angle; }

    // output_to_motors - sends output to named servos
    void output_to_motors() override;

protected:

    // calculate motor outputs - dual-axis specific mixing strategy
    void output_armed_stabilizing() override;

    const char* _get_frame_string() const override { return "TILTROTOR_DUALAXIS"; }

    // spin a motor at the pwm value specified
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // most recent Axis 1 (elbow) tilt position; see set_elbow_tilt_angle()
    float _elbow_tilt_angle = 0.0f;
};
