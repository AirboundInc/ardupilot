#include "tiltrotor.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED
const AP_Param::GroupInfo Tiltrotor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tiltrotor functionality
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables Tiltrotor functionality
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0),

    // @Param: RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40),

    // @Param: MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45),

    // @Param: TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrotor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter,4:DualAxis
    AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS),

    // @Param: RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0),

    // @Param: YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output (fully back). This needs to be set in addition to Q_TILT_TYPE=2, to enable vectored control for yaw in tilt quadplanes. This is also used to limit the forward travel of bicopter tilts(Q_TILT_TYPE=3) when in VTOL modes.
    // @Range: 0 30
    AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0),

    // @Param: FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0),

    // @Param: FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0),

    // @Param: WING_FLAP
    // @DisplayName: Tiltrotor tilt angle that will be used as flap
    // @Description: For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables
    // @Units: deg
    // @Increment: 1
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0),

    // @Param: HVGAIN
    // @DisplayName: Hover tiltrotor vectoring gain
    // @Description: how strongly the attitude vectoring acts in hover (0-1)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("HVGAIN", 11, Tiltrotor, vectoring_gain_hvr, 0),

    // @Param: FWGAIN
    // @DisplayName: Fixed wing tiltrotor vectoring gain
    // @Description: how strongly the attitude vectoring acts in fixed wing flight (0-1)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FWGAIN", 12, Tiltrotor, vectoring_gain_fw, 0),

    // 13 and 14 were BTDLY_MS and FWHLD_MS, the dual axis back transition
    // throttle hold and blend times. Do not reuse

    // @Param: FWHLD_EN
    // @DisplayName: Fixed wing yaw control during back transition
    // @Description: Dual axis tiltrotor: run the fixed wing yaw controller on the rudder through the back transition, until the tilt reaches vertical
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FWHLD_EN", 15, Tiltrotor, fw_control_hold_en, 0),

    // 16 to 19 were THR_BT, THR_FT, FTHLD_MS and FTBLD_MS, the dual axis
    // time based transition throttle hold and blend. Do not reuse

    // @Param: HVPOW
    // @DisplayName: Tiltrotor vector thrust gain power in hover
    // @Description: Power-law exponent applied to the normalized pitch error driving axis 2's extra hover vectoring correction. Values above 1 suppress correction for small pitch errors (avoiding twitchy/windup-prone behavior, e.g. on the ground before takeoff) while still reaching full correction authority at large errors. Negative disables the extra correction entirely.
    // @Range: 0 4
    // @Increment: 0.1
    AP_GROUPINFO("HVPOW", 20, Tiltrotor, vectored_hover_power, 1.0),

    AP_GROUPEND
};

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */

Tiltrotor::Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Tiltrotor::setup()
{

    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER) || (type == TILT_TYPE_DUAL_AXIS) )) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    quadplane.thrust_type = QuadPlane::ThrustType::TILTROTOR;

    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;

    // true if a fixed forward motor is configured, either throttle, throttle left  or throttle right.
    // bicopter tiltrotors use throttle left and right as tilting motors, so they don't count in that case.
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
                    ((SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) || SRV_Channels::function_assigned(SRV_Channel::k_throttleRight))
                        && (type != TILT_TYPE_BICOPTER) && (type != TILT_TYPE_DUAL_AXIS));


    // check if there are any permanent VTOL motors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && !is_motor_tilting(i)) {
            // enabled motor not set in tilt mask
            _have_vtol_motor = true;
            break;
        }
    }

    if (_is_vectored) {
        // we will be using vectoring for yaw
        motors->disable_yaw_torque();
    }

    if (type == TILT_TYPE_DUAL_AXIS) {
        // Set the range for Axis 2 servo channels
        SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeftVec,  4500);
        SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRightVec, 4500);
        // Tell the motor library not to use yaw torque (we'll vector it)
    motors->disable_yaw_torque();
    }

    if (tilt_mask != 0) {
        // setup tilt compensation
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
        if (type == TILT_TYPE_VECTORED_YAW) {
            // setup tilt servos for vectored yaw
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }

    if (type == TILT_TYPE_DUAL_AXIS) {
        dual_axis_transition = NEW_NOTHROW Tiltrotor_Transition_DualAxis(quadplane, motors, *this);
        quadplane.transition = dual_axis_transition;
    } else {
        slt_transition = NEW_NOTHROW Tiltrotor_Transition(quadplane, motors, *this);
        quadplane.transition = slt_transition;
    }
    if (quadplane.transition == nullptr) {
        AP_BoardConfig::allocation_error("tiltrotor transition");
    }

    setup_complete = true;
}

/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const
{
    float rate;
    if (up || max_rate_down_dps <= 0) {
        rate = max_rate_up_dps;
    } else {
        rate = max_rate_down_dps;
    }
    if (type != TILT_TYPE_BINARY && !up && !in_flap_range) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (plane.arming.is_armed_and_safety_off() && !quadplane.in_vtol_mode() && !quadplane.assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt * (1/90.0);
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void Tiltrotor::slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<current_tilt, newtilt > get_fully_forward_tilt());
    current_tilt = constrain_float(newtilt, current_tilt-max_change, current_tilt+max_change);

    angle_achieved = is_equal(newtilt, current_tilt);

    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}

// return the current tilt value that represents forward flight
// tilt wings can sustain forward flight with some amount of wing tilt
float Tiltrotor::get_fully_forward_tilt() const
{
    return 1.0 - (flap_angle_deg * (1/90.0));
}

// return the target tilt value for forward flight
float Tiltrotor::get_forward_flight_tilt() const
{
    return 1.0 - ((flap_angle_deg * (1/90.0)) * SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto) * 0.01);
}

/*
  true if the forward transition has progressed far enough that the tilt
  should go all the way forward, rather than staying limited to Q_TILT_MAX.

  A dual axis tiltrotor reaches this only while assisting during a
  QPOS_AIRBRAKE landing phase: continuous_update() returns early for it
  whenever we are not in a VTOL mode, which is the only time a forward
  transition stage is active.
 */
bool Tiltrotor::transition_tilt_fully_fwd() const
{
    if (dual_axis_transition != nullptr) {
        return dual_axis_transition->active_frwd();
    }
    return slt_transition->transition_state >= Tiltrotor_Transition::TRANSITION_TIMER;
}

/*
  update motor tilt for continuous tilt servos
 */
void Tiltrotor::continuous_update(void)
{
    // default to inactive
    _motors_active = false;

    // the maximum rate of throttle change
    float max_change;

    //if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight || type == TILT_TYPE_DUAL_AXIS)) {
   
    
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor

        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());

        max_change = tilt_max_change(false);

        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (current_tilt < get_fully_forward_tilt()) {
            current_throttle = constrain_float(new_throttle,
                                                    current_throttle-max_change,
                                                    current_throttle+max_change);
        } else {
            current_throttle = new_throttle;
        }
        if (!plane.arming.is_armed_and_safety_off()) {
            current_throttle = 0;
        } else {
            // prevent motor shutdown
            _motors_active = true;
        }
        if (!quadplane.motor_test.running) {
            // the motors are all the way forward, start using them for fwd thrust
            const uint16_t mask = is_zero(current_throttle)?0U:tilt_mask.get();
            motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<current_throttle);
    current_throttle = constrain_float(motors_throttle,
                                            current_throttle-max_change,
                                            current_throttle+max_change);

    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 5 strategies we will use:

      1) With use of a forward throttle controlled by Q_FWD_THR_GAIN in
         VTOL modes except Q_AUTOTUNE determined by Q_FWD_THR_USE. We set the angle based on a calculated
         forward throttle.

      2) With manual forward throttle control we set the angle based on the
         RC input demanded forward throttle for QACRO, QSTABILIZE and QHOVER.

      3) Without a RC input or calculated forward throttle value, the angle
         will be set to zero in QAUTOTUNE, QACRO, QSTABILIZE and QHOVER.
         This enables these modes to be used as a safe recovery mode.

      4) In fixed wing assisted flight or velocity controlled modes we will
         set the angle based on the demanded forward throttle, with a maximum
         tilt given by Q_TILT_MAX. This relies on Q_FWD_THR_GAIN or Q_VFWD_GAIN
         being set.

      5) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */

    if (dual_axis_transition != nullptr &&
        dual_axis_transition->get_stage() == Tiltrotor_Transition_DualAxis::Stage::BACK) {
        // dual axis back transition: straight up at Q_TILT_RATE_UP, ahead of
        // the strategies below, which may hold some forward tilt and so
        // keep the stage from ever ending
        slew(0);
        return;
    }

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        slew(0);
        return;
    }
#endif

    if (!quadplane.assisted_flight &&
        quadplane.get_vfwd_method() == QuadPlane::ActiveFwdThr::NEW &&
        quadplane.is_flying_vtol())
    {
        // We are using the rotor tilt functionality controlled by Q_FWD_THR_GAIN which can
        // operate in all VTOL modes except Q_AUTOTUNE. Forward rotor tilt is used to produce
        // forward thrust equivalent to what would have been produced by a forward thrust motor
        // set to quadplane.forward_throttle_pct()
        const float fwd_g_demand = 0.01 * quadplane.forward_throttle_pct();
        const float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), (float)max_angle_deg);
        slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
        return;
    } else if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover ||
                plane.control_mode == &plane.mode_qloiter ||
                plane.control_mode == &plane.mode_qland ||
                plane.control_mode == &plane.mode_qrtl ||
                plane.control_mode == &plane.mode_auto
            ))
    {
        //set tilt angle to 0
        slew(0);
        return;
    }

    if (quadplane.assisted_flight && transition_tilt_fully_fwd()) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        slew(get_forward_flight_tilt());
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
       float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
       slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void Tiltrotor::binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    float max_change = tilt_max_change(!forward);
    if (forward) {
        current_tilt = constrain_float(current_tilt+max_change, 0, 1);
    } else {
        current_tilt = constrain_float(current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
void Tiltrotor::binary_update(void)
{
    // motors always active
    _motors_active = true;

    if (!quadplane.in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (current_tilt >= 1) {
            const uint16_t mask = is_zero(new_throttle)?0U:tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        binary_slew(false);
    }
}


/*
  update motor tilt
 */
void Tiltrotor::update(void)
{
    //if (!enabled() || tilt_mask == 0) {
    if (!enabled() || (tilt_mask == 0 && type != TILT_TYPE_DUAL_AXIS && type != TILT_TYPE_BICOPTER)) {
        // no motors to tilt
        return;
    }

    if (type == TILT_TYPE_BINARY) {
        binary_update();
    } else {
        continuous_update();
    }

    if (type == TILT_TYPE_VECTORED_YAW) {
        vectoring();
    }
}

#if HAL_LOGGING_ENABLED
// Write tiltrotor specific log
void Tiltrotor::write_log()
{
    // Only valid on a tiltrotor
    if (!enabled()) {
        return;
    }

    struct log_tiltrotor pkt {
        LOG_PACKET_HEADER_INIT(LOG_TILT_MSG),
        time_us      : AP_HAL::micros64(),
        current_tilt : current_tilt * 90.0,
        stage        : dual_axis_transition ? uint8_t(dual_axis_transition->get_stage()) : uint8_t(0),
        airspeed     : plane.logger.quiet_nanf(),
        target_airspeed : float(plane.aparm.airspeed_min),
        trans_throttle : dual_axis_transition ? dual_axis_transition->trans_throttle_pct : 0,
    };
    float aspeed;
    if (plane.ahrs.airspeed_estimate(aspeed)) {
        pkt.airspeed = aspeed;
    }

    if (type != TILT_TYPE_VECTORED_YAW) {
        // Left and right tilt are invalid
        pkt.front_left_tilt = plane.logger.quiet_nanf();
        pkt.front_right_tilt = plane.logger.quiet_nanf();

    } else {
        // Calculate tilt angle from servo outputs
        const float total_angle = 90.0 + tilt_yaw_angle + fixed_angle;
        const float scale = total_angle * 0.001;
        pkt.front_left_tilt = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft) * scale) - tilt_yaw_angle;
        pkt.front_right_tilt = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * scale) - tilt_yaw_angle;
    }

    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif

/*
  tilt compensation for angle of tilt. When the rotors are tilted the
  roll effect of differential thrust on the tilted rotors is decreased
  and the yaw effect increased
  We have two factors we apply.

  1) when we are transitioning to fwd flight we scale the tilted rotors by 1/cos(angle). This pushes us towards more flight speed

  2) when we are transitioning to hover we scale the non-tilted rotors by cos(angle). This pushes us towards lower fwd thrust

  We also apply an equalisation to the tilted motors in proportion to
  how much tilt we have. This smoothly reduces the impact of the roll
  gains as we tilt further forward.

  For yaw, we apply differential thrust in proportion to the demanded
  yaw control and sin of the tilt angle

  Finally we ensure no requested thrust is over 1 by scaling back all
  motors so the largest thrust is at most 1.0
 */
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul)
{
    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factors first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= non_tilted_mul;
        } else {
            thrust[i] *= tilted_mul;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;
    const float sin_tilt = sinf(radians(current_tilt*90));
    // yaw_gain relates the amount of differential thrust we get from
    // tilt, so that the scaling of the yaw control is the same at any
    // tilt angle
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    const float avg_tilt_thrust = tilt_total / tilt_count;

    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            // as we tilt we need to reduce the impact of the roll
            // controller. This simple method keeps the same average,
            // but moves us to no roll control as the angle increases
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
            // add in differential thrust for yaw control, scaled by tilt angle
            const float diff_thrust = motors->get_roll_factor(i) * (motors->get_yaw()+motors->get_yaw_ff()) * sin_tilt * yaw_gain;
            thrust[i] += diff_thrust;
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the motors then reduce all motors
    // to keep them in proportion to the original thrust. This helps
    // maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (quadplane.in_vtol_mode()) {
        // we are transitioning to VTOL flight
        const float tilt_factor = cosf(radians(current_tilt*90));
        tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
    } else {
        float inv_tilt_factor;
        if (current_tilt > 0.98f) {
            inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
        } else {
            inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
        }
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}

/*
  return true if the rotors are fully tilted forward
 */
bool Tiltrotor::fully_fwd(void) const
{
    /*
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
    */
    if (!enabled()) {
        return false;
    }
    if (tilt_mask == 0 && type != TILT_TYPE_DUAL_AXIS && type != TILT_TYPE_BICOPTER) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
}


/*
  return true if the rotors are fully tilted up
 */
bool Tiltrotor::fully_up(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt <= 0);
}

/*
  control vectoring for tilt multicopters
 */
void Tiltrotor::vectoring(void)
{
    // total angle the tilt can go through
    const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
    // output value (0 to 1) to get motors pointed straight up
    const float zero_out = tilt_yaw_angle / total_angle;
    const float fixed_tilt_limit = fixed_angle / total_angle;
    const float level_out = 1.0 - fixed_tilt_limit;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out + (current_tilt * (level_out - zero_out));
    // for testing when disarmed, apply vectored yaw in proportion to rudder stick
    // Wait TILT_DELAY_MS after disarming to allow props to spin down first.
    constexpr uint32_t TILT_DELAY_MS = 3000;
    uint32_t now = AP_HAL::millis();
    if (!plane.arming.is_armed_and_safety_off() && plane.quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT)) {
        // this test is subject to wrapping at ~49 days, but the consequences are insignificant
        if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
            if (quadplane.in_vtol_mode()) {
                float yaw_out = plane.channel_rudder->get_control_in();
                yaw_out /= plane.channel_rudder->get_range();
                float yaw_range = zero_out;

                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
            } else {
                // fixed wing tilt
                const float gain = fixed_gain * fixed_tilt_limit;
                // base the tilt on elevon mixing, which means it
                // takes account of the MIXING_GAIN. The rear tilt is
                // based on elevator
                const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
                const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
                const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
                // front tilt is effective canards, so need to swap and use negative. Rear motors are treated live elevons.
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
            }
        }
        return;
    }

    const bool no_yaw = tilt_over_max_angle();
    if (no_yaw) {
        // fixed wing  We need to apply inverse scaling with throttle, and remove the surface speed scaling as
        // we don't want tilt impacted by airspeed
        const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
        const float gain = fixed_gain * fixed_tilt_limit * scaler;
        const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
        const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
        const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
    } else {
        const float yaw_out = motors->get_yaw()+motors->get_yaw_ff();
        const float roll_out = motors->get_roll()+motors->get_roll_ff();
        const float yaw_range = zero_out;

        // Scaling yaw with throttle
        const float throttle = motors->get_throttle_out();
        const float scale_min = 0.5;
        const float scale_max = 2.0;
        float throttle_scaler = scale_max;
        if (is_positive(throttle)) {
            throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
        }

        // now apply vectored thrust for yaw and roll.
        const float tilt_rad = radians(current_tilt*90);
        const float sin_tilt = sinf(tilt_rad);
        const float cos_tilt = cosf(tilt_rad);
        // the MotorsMatrix library normalises roll factor to 0.5, so
        // we need to use the same factor here to keep the same roll
        // gains when tilted as we have when not tilted
        const float avg_roll_factor = 0.5;
        float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

        if (fabsf(tilt_scale) > 1.0) {
            tilt_scale = constrain_float(tilt_scale, -1.0, 1.0);
            motors->limit.yaw = true;
        }

        const float tilt_offset = tilt_scale * yaw_range;

        float left_tilt = base_output + tilt_offset;
        float right_tilt = base_output - tilt_offset;

        // if output saturation of both left and right then set yaw limit flag
        if (((left_tilt > 1.0) || (left_tilt < 0.0)) &&
            ((right_tilt > 1.0) || (right_tilt < 0.0))) {
            motors->limit.yaw = true;
        }

        // constrain and scale to ouput range
        left_tilt = constrain_float(left_tilt,0.0,1.0) * 1000.0;
        right_tilt = constrain_float(right_tilt,0.0,1.0) * 1000.0;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 1000.0 * constrain_float(base_output,0.0,1.0));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, right_tilt);
    }
}

/*
  control bicopter tiltrotors
 */
void Tiltrotor::bicopter_output(void)
{
    if (type != TILT_TYPE_BICOPTER || quadplane.motor_test.running) {
        // don't override motor test with motors_output
        return;
    }

    if (!quadplane.in_vtol_mode() && fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (quadplane.assisted_flight) {
        quadplane.hold_stabilize(throttle * 0.01f);
        quadplane.motors_output(true);
    } else {
        quadplane.motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt_yaw_angle * (1/90.0);
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt_yaw_angle * (1/90.0);
    }

    // reduce authority of bicopter as motors are tilted forwards
    const float scaling = cosf(current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    tilt_left  = constrain_float(-(current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}

/*
  control dual-axis tiltrotor. Axis 1 (V22-style 0-90 transition tilt)
  is driven by continuous_update(). This function handles Axis 2
  (independent attitude vectoring servos) called from servos.cpp.

  The base roll/pitch/yaw/throttle mix for the two lift motors, and the
  raw Axis 2 vectoring demand, are computed by AP_MotorsTiltrotorDualAxis
  (via quadplane.motors_output() inside update_controllers() above, which
  writes k_throttleLeft/Right and the raw vectoring signal to
  k_tiltMotorLeftVec/RightVec). This function reads that raw vectoring
  signal, layers pitch-error feedback and transition blending on top, and
  writes the final Axis 2 output.
 */
void Tiltrotor::dual_axis_output(void)
{
    // setup() allocates dual_axis_transition at boot, so it is null when
    // Q_TILT_TYPE has been changed to DualAxis since; the dual axis outputs
    // stay off until the reboot that Q_TILT_TYPE requires
    if (type != TILT_TYPE_DUAL_AXIS || dual_axis_transition == nullptr || quadplane.motor_test.running) {
        return;
    }

    const float axis1_pos = -(current_tilt * SERVO_MAX);
    const float elbow_position =  radians(-axis1_pos*90.0f/SERVO_MAX);
    // Set elbow angle to the motor mixer.
    static_cast<AP_MotorsTiltrotorDualAxis*>(motors)->set_elbow_tilt_angle(elbow_position);
    // motors is guaranteed to be an AP_MotorsTiltrotorDualAxis instance here:
    // quadplane.cpp only allocates that class when type == TILT_TYPE_DUAL_AXIS
    // the VTOL mixer runs in VTOL flight and through both transitions,
    // the FW mixer only once the forward transition has reached
    // AIRSPEED_MIN
    dual_axis_transition->update_controllers();
    dual_axis_mixout_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (dual_axis_transition->get_stage() == Tiltrotor_Transition_DualAxis::Stage::FW) {
        run_fw_mixer();
    } else {
        run_vtol_mixer();
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  axis1_pos);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, axis1_pos);
    return;
}

/*
  when doing a forward transition of a tilt-vectored quadplane we use
  euler angle control to maintain good yaw. This updates the yaw
  target based on pilot input and target roll
 */
void Tiltrotor::update_yaw_target(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - transition_yaw_set_ms > 100 ||
        !is_zero(quadplane.get_pilot_input_yaw_rate_cds())) {
        // lock initial yaw when transition is started or when
        // pilot commands a yaw change. This allows us to track
        // straight in transitions for tilt-vectored planes, but
        // allows for turns when level transition is not wanted
        transition_yaw_cd = quadplane.ahrs.yaw_sensor;
    }

    /*
      now calculate the equivalent yaw rate for a coordinated turn for
      the desired bank angle given the airspeed
     */
    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
        float dt = (now - transition_yaw_set_ms) * 0.001;
        // calculate the yaw rate to achieve the desired turn rate
        const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
        const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
        transition_yaw_cd += yaw_rate_cds * dt;
    }
    transition_yaw_set_ms = now;
}


/*
  control use of multirotor rate control in forward transition
 */
bool Tiltrotor_Transition::use_multirotor_control_in_fwd_transition() const
{
    return tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER;
}

bool Tiltrotor_Transition::update_yaw_target(float& yaw_target_cd)
{
    if (!use_multirotor_control_in_fwd_transition()) {
        return false;
    }
    tiltrotor.update_yaw_target();
    yaw_target_cd = tiltrotor.transition_yaw_cd;
    return true;
}

// return true if we should show VTOL view
bool Tiltrotor_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (!show_vtol && tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER) {
        // we use multirotor controls during fwd transition for
        // vectored yaw vehicles
        return true;
    }

    return show_vtol;
}

// return true if we are tilted over the max angle threshold
bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}


/*
  ================== Tiltrotor_Transition_DualAxis ==================
 */

void Tiltrotor_Transition_DualAxis::set_stage(Stage new_stage)
{
    if (new_stage == stage) {
        return;
    }
    stage = new_stage;

    switch (new_stage) {
    case Stage::VTOL:
        gcs().send_text(MAV_SEVERITY_INFO, "Back trans done");
        break;
    case Stage::BACK:
        gcs().send_text(MAV_SEVERITY_INFO, "Back trans: tilting up%s",
                         tiltrotor.fw_control_hold_en > 0 ? " (FW yaw ctrl)" : "");
        break;
    case Stage::FWD:
        gcs().send_text(MAV_SEVERITY_INFO, "Fwd trans: waiting for %.1f m/s",
                         float(plane.aparm.airspeed_min));
        break;
    case Stage::FW:
        gcs().send_text(MAV_SEVERITY_INFO, "Fwd trans done");
        break;
    }
}

bool Tiltrotor_Transition_DualAxis::airspeed_reached() const
{
    float aspeed;
    return plane.ahrs.airspeed_estimate(aspeed) && aspeed >= plane.aparm.airspeed_min;
}

/*
  forward-side driver, called every loop from QuadPlane::update() while
  !in_vtol_mode(). Starts FWD on the VTOL->FW edge and completes to FW once
  airspeed reaches AIRSPEED_MIN. Without an airspeed estimate it stays in
  FWD, on the VTOL mixer.
 */
void Tiltrotor_Transition_DualAxis::update()
{
    if (stage == Stage::VTOL || stage == Stage::BACK) {
        // first tick since leaving VTOL mode: start a fresh forward transition
        set_stage(Stage::FWD);
    }

    if (stage == Stage::FWD && airspeed_reached() && tiltrotor.fully_fwd()) {
        set_stage(Stage::FW);
    }

    quadplane.assisted_flight = (stage == Stage::FWD);

    if (stage != Stage::FW) {
        // TECS uses a synthetic airspeed through the forward transition:
        // pitch limits would otherwise throw off its throttle calculation,
        // which is driven by pitch, before real airspeed has built up
        plane.TECS_controller.use_synthetic_airspeed();
    }

    set_last_fw_pitch();
}

/*
  VTOL-side driver, called every loop from QuadPlane::update() while
  in_vtol_mode(). Starts BACK on the FW->VTOL edge and completes to VTOL
  once the tilt, coming up at Q_TILT_RATE_UP, reaches vertical. Airspeed
  does not gate it: slowing down is done in fixed wing flight before the
  switch, by the pilot or by FW RTL holding AIRSPEED_MIN.
 */
void Tiltrotor_Transition_DualAxis::VTOL_update()
{
    const uint32_t now = AP_HAL::millis();

    if (stage == Stage::FW || stage == Stage::FWD) {
        if (last_fw_output_ms != 0 && (now - last_fw_output_ms) < FW_OUTPUT_TIMEOUT_MS) {
            // first tick since leaving FW mode: start a fresh back transition
            set_stage(Stage::BACK);
        } else {
            // no recent fixed wing outputs, so this is a boot, a forced
            // stage reset or a forward transition abandoned before reaching
            // AIRSPEED_MIN, rather than a back transition
            stage = Stage::VTOL;
            return;
        }
    }

    if (stage == Stage::BACK && is_zero(tiltrotor.current_tilt)) {
        // Tiltrotor::update() slews the tilt after this, so this sees last
        // tick's tilt and the stage ends one tick after it reaches vertical
        set_stage(Stage::VTOL);
    }

    if (hold_level_attitude() && quadplane.pos_control->is_active_xy()) {
        // the mode's run() has already updated the position controller this
        // tick; reset its XY targets to the current state and decay its
        // integrator so it resumes from where the vehicle is when the hold
        // ends. The Z controller keeps running through the transition
        quadplane.pos_control->relax_velocity_controller_xy();
    }

    // assistance stays reset while it is not being checked
    quadplane.assist.reset();
}

/*
  true while a transition stage holds roll and pitch at zero and the yaw
  rate at zero, for both the VTOL and the fixed wing controllers
 */
bool Tiltrotor_Transition_DualAxis::hold_level_attitude() const
{
    switch (stage) {
    case Stage::FWD:
    case Stage::BACK:
        return true;
    case Stage::VTOL:
    case Stage::FW:
        break;
    }
    return false;
}

void Tiltrotor_Transition_DualAxis::restart()
{
    stage = Stage::VTOL;
}

void Tiltrotor_Transition_DualAxis::force_transition_complete()
{
    stage = Stage::FW;
    set_last_fw_pitch();
    quadplane.assist.reset();
}

// a dual axis tiltrotor uses fixed wing controls as soon as it leaves a
// VTOL mode, so the VTOL view follows the flight mode directly
bool Tiltrotor_Transition_DualAxis::show_vtol_view() const
{
    return quadplane.in_vtol_mode();
}

MAV_VTOL_STATE Tiltrotor_Transition_DualAxis::get_mav_vtol_state() const
{
    switch (stage) {
    case Stage::VTOL:
        return MAV_VTOL_STATE_MC;
    case Stage::BACK:
        return MAV_VTOL_STATE_TRANSITION_TO_MC;
    case Stage::FWD:
        return MAV_VTOL_STATE_TRANSITION_TO_FW;
    case Stage::FW:
        return MAV_VTOL_STATE_FW;
    }
    return MAV_VTOL_STATE_UNDEFINED;
}

/*
  run the FW yaw controller through the BACK stage when Q_TILT_FWHLD_EN is
  set. Q-modes centre the rudder rather than calling stabilize_yaw(), and
  dual_axis_output() runs after them, so this is what puts the rudder
  under yaw control for the back transition.
 */
void Tiltrotor_Transition_DualAxis::update_yaw_authority() const
{
    if (stage == Stage::BACK && tiltrotor.fw_control_hold_en > 0) {
        plane.stabilize_yaw();
    }
}

/*
  run the controllers the current stage calls for and return the ESC
  throttle percentage to command. Called once per loop from
  Tiltrotor::run_vtol_mixer(), so only outside the FW stage.
 */
void Tiltrotor_Transition_DualAxis::update_controllers(void)
{
    switch (stage) {
    case Stage::VTOL:
        // steady hover: QuadPlane::update() already ran the rate controller
        // this tick, so this pushes out that mix without recomputing it
        quadplane.motors_output(false);
        break;

    case Stage::BACK:
        quadplane.motors_output(false);
        update_yaw_authority();
        quadplane.set_climb_rate_cms(0);
        break;

    case Stage::FWD:
        // To hold level VTOL attitude control need to active in Foward transition.
        quadplane.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        plane.nav_roll_cd, plane.nav_pitch_cd, 0);
        // VTOL z controller.
        quadplane.run_z_controller();
        quadplane.motors_output(true);
        break;

    case Stage::FW:
        break;
    }
}

void Tiltrotor_Transition_DualAxis::set_last_fw_pitch(void)
{
    last_fw_mode_ms = AP_HAL::millis();
    last_fw_nav_pitch_cd = plane.nav_pitch_cd;
}

/*
  limit VTOL roll and pitch during the back transition: constrains both to
  Q_ANGLE_MAX, holds pitch down at low airspeed where the control surfaces
  would reverse, and applies an expanding pitch envelope for
  Q_BACKTRANS_MS after entering VTOL flight.
 */
bool Tiltrotor_Transition_DualAxis::set_VTOL_roll_pitch_limit(int32_t& roll_cd, int32_t& pitch_cd)
{
    if (hold_level_attitude()) {
        // level hold for modes that set nav_roll/pitch in run(). Returning
        // true makes the caller mark the XY position controller as limited
        roll_cd = 0;
        pitch_cd = 0;
        return true;
    }

    bool ret = false;
    const int16_t angle_max = quadplane.aparm.angle_max;

    int32_t new_roll_cd = constrain_int32(roll_cd, -angle_max, angle_max);
    if (new_roll_cd != roll_cd) {
        roll_cd = new_roll_cd;
        ret = true;
    }

    if (pitch_cd < -angle_max) {
        pitch_cd = -angle_max;
        ret = true;
    }

    float airspeed;
    if (pitch_cd > angle_max &&
        plane.ahrs.airspeed_estimate(airspeed) && airspeed < 0.5 * plane.aparm.airspeed_min) {
        const float max_limit_cd = linear_interpolate(angle_max, 4500,
                                                      airspeed,
                                                      0, 0.5 * plane.aparm.airspeed_min);
        if (pitch_cd > max_limit_cd) {
            pitch_cd = max_limit_cd;
            ret = true;
        }
    }

    if (quadplane.back_trans_pitch_limit_ms <= 0) {
        return ret;
    }

    const uint32_t limit_time_ms = quadplane.back_trans_pitch_limit_ms;

    const uint32_t dt = AP_HAL::millis() - last_fw_mode_ms;
    if (last_fw_mode_ms == 0 || dt > limit_time_ms) {
        last_fw_mode_ms = 0;
        return ret;
    }

    const float max_limit_cd = linear_interpolate(MAX(last_fw_nav_pitch_cd,0), MIN(angle_max,plane.aparm.pitch_limit_max*100),
                                            dt,
                                            0, limit_time_ms);

    if (pitch_cd > max_limit_cd) {
        pitch_cd = max_limit_cd;
        return true;
    }

    const float min_limit_cd = linear_interpolate(MIN(last_fw_nav_pitch_cd,0), MAX(-angle_max,plane.aparm.pitch_limit_min*100),
                                                  dt,
                                                  0, limit_time_ms);

    if (plane.nav_pitch_cd < min_limit_cd) {
        plane.nav_pitch_cd = min_limit_cd;
        return true;
    }

    return ret;
}

/*
  level hold through the transition stages. Runs ahead of every mode's
  run(), VTOL modes included, so this is the level hold for modes that set
  nav_roll/pitch in update(). Once FW is reached the FW pitch limits are
  the mode's own.
 */
void Tiltrotor_Transition_DualAxis::set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd)
{
    if (!hold_level_attitude()) {
        return;
    }
    nav_roll_cd = 0;
    nav_pitch_cd = 0;
    if (!quadplane.in_vtol_mode()) {
        // keep TECS from winding up against a pitch it cannot have
        plane.TECS_controller.set_pitch_max(0);
        plane.TECS_controller.set_pitch_min(0);
    }
}

void Tiltrotor::run_vtol_mixer(void){

    float tilt_left  = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
    float tilt_left_adjusted = tilt_left;
    float tilt_right_adjusted = tilt_right;

    // drive TVs based on pitch error as well
    float des_pitch_cd = quadplane.attitude_control->get_att_target_euler_cd().y;
    float pitch_cd = quadplane.ahrs_view->pitch_sensor;

    float des_pitch_cd2 = plane.nav_pitch_cd;
    float pitch_cd2 = plane.ahrs.pitch_sensor;

    float pitch_error_cd = (des_pitch_cd - pitch_cd) * vectoring_gain_hvr;

    float extra_pitch = constrain_float(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
    float extra_sign = extra_pitch > 0?1:-1;
    float extra_elevator = 0;
    bool is_vtol = quadplane.in_vtol_mode();

    if (!is_zero(extra_pitch) && is_vtol && !is_negative(vectored_hover_power)) {
        extra_elevator = extra_sign * powf(fabsf(extra_pitch), vectored_hover_power) * SERVO_MAX;
    }

    tilt_left_adjusted  += extra_elevator;
    tilt_right_adjusted += extra_elevator;

#if HAL_LOGGING_ENABLED
    // Add logging for desired thrust vectoring angles
    AP::logger().WriteStreaming("PHID", "TimeUS,DesL,DesR,ExtraEl,AdjL,AdjR,PitchErr,isVTOL,ExtraPit",
            "sdddddddd", // seconds, degrees
            "F00000000", // micro (1e-6), no mult (1e0)
            "Qffffffff", // uint64_t, float
            AP_HAL::micros64(), tilt_left/100, tilt_right/100, extra_elevator/100,
            tilt_left_adjusted/100, 
            tilt_right_adjusted/100,
            pitch_error_cd/100,
            (float)is_vtol,
            extra_pitch/100);

    AP::logger().WriteStreaming("PITE", "TimeUS,DesPit1,DesPit2,Pit1,Pit2",
            "sdddd", // seconds, degrees
            "F0000", // micro (1e-6), no mult (1e0)
            "Qffff", // uint64_t, float
            AP_HAL::micros64(), des_pitch_cd/100, des_pitch_cd2/100,pitch_cd/100,pitch_cd2/100);
#endif
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeftVec,
                                    constrain_float(tilt_left_adjusted,  -SERVO_MAX, SERVO_MAX));
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRightVec,
                                    constrain_float(tilt_right_adjusted, -SERVO_MAX, SERVO_MAX));

}
void Tiltrotor::run_fw_mixer(void){
    dual_axis_transition->note_fw_output();

    // first FW mixer tick after VTOL/transition: restart the DT ramp and
    // drop any yaw I built up while differential thrust wasn't applied
    const uint32_t now = AP_HAL::millis();
    if (now - last_fw_mixer_ms > 100) {
        fw_entry_ms = now;
        plane.yawController.reset_I();
    }
    last_fw_mixer_ms = now;

    // never command motor throttle while disarmed: this branch runs
    // unconditionally in FW modes (even disarmed on the ground, since
    // assisted_flight is false here), and would otherwise write raw
    // stick/k_throttle straight to the ESCs, undoing the disarm safety-zero
    // Plane::set_servos() already applies earlier in the same tick
    const float throttle = !plane.arming.is_armed_and_safety_off() ? 0.0f
        : plane.control_mode->does_auto_throttle()
        ? SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        : plane.get_throttle_input(true);

    // ramp differential thrust in over DT_RAMP_MS after entering FW so any
    // rudder demand present at handover isn't applied as a step
    constexpr float DT_RAMP_MS = 500.0f;
    const float dt_ramp   = constrain_float((now - fw_entry_ms) / DT_RAMP_MS, 0.0f, 1.0f);
    const float rud_gain  = float(plane.g2.rudd_dt_gain) * 0.01f;
    const float rudder_dt = dt_ramp * rud_gain * SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * (1.0f / SERVO_MAX);

    // DT is the only yaw actuator: near full throttle give up collective
    // rather than clipping one side of the differential
    const float half_split = fabsf(50.0f * rudder_dt);
    const float thr = MIN(throttle, 100.0f - half_split);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  constrain_float(thr + 50.0f * rudder_dt, 0, 100));
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, constrain_float(thr - 50.0f * rudder_dt, 0, 100));


    // forward flight: Axis 1 is at 90deg (motors fully forward)
    // use rudder for differential yaw vectoring via Axis 2
    // set Q_TILT_VEC_FWGAIN > 0 to enable; default 0 disables it
    if (!is_positive(vectoring_gain_fw)) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeftVec,  0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRightVec, 0);
        return;
    }
    const float scaler = (plane.control_mode == &plane.mode_manual) ? 1.0f :
                         (quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
    const float gain   = vectoring_gain_fw * scaler;
    const float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1.0f / 4500.0f);
    const float aileron  = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)  * (1.0f / 4500.0f);

    float tilt_left = constrain_float((elevator + aileron) * gain, -1.0f, 1.0f) * SERVO_MAX;
    float tilt_right = constrain_float((elevator - aileron) * gain, -1.0f, 1.0f) * SERVO_MAX;

#if HAL_LOGGING_ENABLED
        // Add logging for desired thrust vectoring angles
        AP::logger().WriteStreaming("PHIF", "TimeUS,DesL,DesR",
                "sdd", // seconds, degrees
                "F00", // micro (1e-6), no mult (1e0)
                "Qff", // uint64_t, float
                AP_HAL::micros64(), tilt_left/100, tilt_right/100);
#endif

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeftVec, tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRightVec, tilt_right);
}

#endif  // HAL_QUADPLANE_ENABLED
