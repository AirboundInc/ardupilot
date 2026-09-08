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
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <AP_Logger/LogStructure.h>

class QuadPlane;
class AP_MotorsMulticopter;
class Tiltrotor_Transition;
class Tiltrotor_Transition_DualAxis;
class Tiltrotor
{
friend class QuadPlane;
friend class Plane;
friend class Tiltrotor_Transition;
friend class Tiltrotor_Transition_DualAxis;
public:

    Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    bool enabled() const { return (enable > 0) && setup_complete;}

    void setup();

    void slew(float tilt);
    void binary_slew(bool forward);
    void update();
    void continuous_update();
    void binary_update();
    void vectoring();
    void bicopter_output();
    void dual_axis_output();

    // most recent k_throttle value written by AP_MotorsTailsitter's mixer
    // inside dual_axis_output(), before it gets restored, for debug logging
    float get_last_dual_axis_mixout_throttle() const { return dual_axis_mixout_throttle; }

    void tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool tilt_over_max_angle(void) const;

    bool is_motor_tilting(uint8_t motor) const {
        return tilt_mask.get() & (1U<<motor);
    }

    bool fully_fwd() const;
    bool fully_up() const;
    float tilt_max_change(bool up, bool in_flap_range = false) const;
    float get_fully_forward_tilt() const;
    float get_forward_flight_tilt() const;

    // update yaw target for tiltrotor transition
    void update_yaw_target();

    bool is_vectored() const { return enabled() && _is_vectored; }

    bool has_fw_motor() const { return _have_fw_motor; }

    bool has_vtol_motor() const { return _have_vtol_motor; }

    bool motors_active() const { return enabled() && _motors_active; }

    // true if the tilts have completed slewing
    // always return true if not enabled or not a continuous type
    bool tilt_angle_achieved() const { return !enabled() || (type != TILT_TYPE_CONTINUOUS) || angle_achieved; }

    // Write tiltrotor specific log
    void write_log();

    AP_Int8 enable;
    AP_Int16 tilt_mask;
    AP_Int16 max_rate_up_dps;
    AP_Int16 max_rate_down_dps;
    AP_Int8  max_angle_deg;
    AP_Int8  type;
    AP_Float tilt_yaw_angle;
    AP_Float fixed_angle;
    AP_Float fixed_gain;
    AP_Float flap_angle_deg;
    AP_Float vectoring_gain_hvr;
    AP_Float vectored_hover_power;
    AP_Float vectoring_gain_fw;

    // Time to blend from held FW throttle to pilot throttle after the
    // fw_throttle_hold_ms hold period, following a backtransition into
    // VTOL mode (dual axis tiltrotor)
    AP_Float back_trans_delay_ms;

    // Time to hold the last FW throttle steady after backtransition into
    // VTOL mode, before blending to pilot throttle over back_trans_delay_ms
    // (dual axis tiltrotor)
    AP_Float fw_throttle_hold_ms;

    // Enable/Disable to hold fixed wing controller during fw_throttle_hold_ms in back transition
    AP_Int8 fw_control_hold_en;

    // Fixed throttle (percent) to hold during fw_throttle_hold_ms after a
    // backtransition, instead of the last fixed wing throttle
    AP_Float back_trans_hold_throttle;

    // Fixed throttle (percent) to hold during fwd_trans_hold_ms at the start
    // of a forward transition (dual axis tiltrotor), before blending to the
    // FBWA/commanded throttle over fwd_trans_blend_ms
    AP_Float fwd_trans_hold_throttle;

    // Time to hold fwd_trans_hold_throttle steady at the start of a forward
    // transition into fixed wing flight, before blending to the
    // FBWA/commanded throttle (dual axis tiltrotor)
    AP_Float fwd_trans_hold_ms;

    // Time to blend from the held fwd_trans_hold_throttle to the
    // FBWA/commanded throttle after the fwd_trans_hold_ms hold period,
    // during a forward transition (dual axis tiltrotor)
    AP_Float fwd_trans_blend_ms;

    float current_tilt;
    float current_throttle;
    bool _motors_active:1;
    float transition_yaw_cd;
    uint32_t transition_yaw_set_ms;
    bool _is_vectored;

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS    =0,
          TILT_TYPE_BINARY        =1,
          TILT_TYPE_VECTORED_YAW  =2,
          TILT_TYPE_BICOPTER      =3,
          TILT_TYPE_DUAL_AXIS     =4 // new v22 tilt + thrust vectoring
    };

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Tiltrotor specific log message
    struct PACKED log_tiltrotor {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float current_tilt;
        float front_left_tilt;
        float front_right_tilt;
        uint32_t backtrans_elapsed_ms;
        float fw_throttle;
        float pilot_throttle;
        float blend_throttle;
        uint32_t fwdtrans_elapsed_ms;
        float fwdtrans_commanded_throttle;
        float fwdtrans_blend_throttle;
    };

    bool setup_complete;

    // true if a fixed forward motor is setup
    bool _have_fw_motor;

    // true if all motors tilt with no fixed VTOL motor
    bool _have_vtol_motor;

    // true if the current tilt angle is equal to the desired
    // with slow tilt rates the tilt angle can lag
    bool angle_achieved;

    // refences for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

    // throttle (0 to 1) that was last commanded in fw control mode
    float last_fw_throttle = 0;

    // k_throttle value written by AP_MotorsTailsitter's collective-thrust
    // actuator output inside dual_axis_output(), before it gets restored
    // back to the fixed-wing forward-throttle value; for QTHR debug log
    float dual_axis_mixout_throttle = 0;

    // true if the forward transition has progressed far enough that the
    // tilt should go all the way forward rather than being limited to
    // Q_TILT_MAX
    bool transition_tilt_fully_fwd() const;

    /*
      setup() allocates exactly one of these according to type and hands it
      to QuadPlane as its Transition: dual_axis_transition for
      TILT_TYPE_DUAL_AXIS, slt_transition for every other tilt type. Both
      are held as concrete types so the tilt code can read each machine's
      own state, Stage and TRANSITION_* respectively.
     */
    Tiltrotor_Transition* slt_transition = nullptr;
    Tiltrotor_Transition_DualAxis* dual_axis_transition = nullptr;

};

// Transition for separate left thrust quadplanes
class Tiltrotor_Transition : public SLT_Transition
{
friend class Tiltrotor;
public:

    Tiltrotor_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):SLT_Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {};

    bool update_yaw_target(float& yaw_target_cd) override;

    bool show_vtol_view() const override;

    bool use_multirotor_control_in_fwd_transition() const override;

private:

    // time when we entered VTOL mode from FW (for Q_BTDELAY_MS)
    uint32_t backtrans_start_ms;

    Tiltrotor& tiltrotor;

};

/*
  Transition controller for dual axis tiltrotors (Q_TILT_TYPE=DualAxis).

  Runs its own stage machine rather than SLT_Transition's airspeed wait:
  each transition is a sequence of timed stages started by a VTOL<->FW mode
  edge, with the stage durations coming from Q_TILT_FTHLD_MS/FTBLD_MS
  forwards and Q_TILT_FWHLD_MS/BTDLY_MS back.

  It also selects, per stage, which controllers drive the vehicle and what
  throttle they use: the multicopter attitude/rate controller through
  hold_stabilize()/motors_output(), and the fixed wing yaw controller
  through stabilize_yaw(). Tiltrotor::dual_axis_output() calls
  update_controllers() once per loop and applies the throttle it returns,
  so the stage behaviour is the same in every Q-mode.

  Fixed wing roll and pitch are not driven from here. Every Q-mode already
  calls stabilize_roll()/stabilize_pitch() each tick against whatever set
  plane.nav_roll_cd/nav_pitch_cd that tick, pilot sticks in QSTABILIZE and
  QHOVER or the position controller in QLOITER/QRTL/AUTO/GUIDED, and
  dual_axis_output() runs after that from servos.cpp. A stage that needs to
  command roll or pitch does so from set_VTOL_roll_pitch_limit(), which
  those modes call after computing nav_roll_cd/nav_pitch_cd and before
  stabilize_roll()/stabilize_pitch() consumes them, or from
  set_FW_roll_pitch(), which Plane::stabilize() calls ahead of every mode's
  run() and which acts here only while not in a VTOL mode.
 */
class Tiltrotor_Transition_DualAxis : public Transition
{
friend class Tiltrotor;
public:

    Tiltrotor_Transition_DualAxis(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):
        Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {}

    // stages of the dual axis forward and back transition. Each *_HOLD and
    // *_BLEND pair is a sub-window of one elapsed-time timer, sequenced by
    // update()/VTOL_update() and given its controllers and throttle by
    // update_controllers()
    enum class Stage : uint8_t {
        VTOL,        // steady hover, no transition active
        BACK_HOLD,   // just left FW: hold throttle for Q_TILT_FWHLD_MS, optional FW yaw authority
        BACK_BLEND,  // blend held throttle -> VTOL controller's demand over Q_TILT_BTDLY_MS
        FWD_HOLD,    // just left VTOL: hold throttle for Q_TILT_FTHLD_MS while tilt ramps forward
        FWD_BLEND,   // blend held throttle -> FBWA/commanded throttle over Q_TILT_FTBLD_MS
        FW,          // transition complete, pure fixed wing
    };
    Stage get_stage() const { return stage; }

    void update() override;
    void VTOL_update() override;

    void force_transition_complete() override;
    bool complete() const override { return stage == Stage::FW; }
    void restart() override;
    uint8_t get_log_transition_state() const override { return static_cast<uint8_t>(stage); }
    bool active_frwd() const override { return stage == Stage::FWD_HOLD || stage == Stage::FWD_BLEND; }
    bool show_vtol_view() const override;
    MAV_VTOL_STATE get_mav_vtol_state() const override;
    bool set_VTOL_roll_pitch_limit(int32_t& roll_cd, int32_t& pitch_cd) override;
    void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd) override;
    void set_last_fw_pitch(void) override;

    // records that fixed wing outputs are being driven this loop, which is
    // what qualifies a following VTOL mode as a back transition
    void note_fw_output() { last_fw_output_ms = AP_HAL::millis(); }

    // Runs the controllers the current stage calls for and returns the ESC
    // throttle percentage to command. Called once per loop from
    // Tiltrotor::dual_axis_output(), after the active Q-mode has set
    // plane.nav_roll_cd/nav_pitch_cd and its own attitude and throttle
    // targets. Both throttles are 0-100 percent.
    float update_controllers(float pilot_vtol_throttle_pct, float commanded_fw_throttle_pct);

private:

    void set_stage(Stage new_stage);

    // hold the Q_TILT_THR_BT throttle steady for Q_TILT_FWHLD_MS, then
    // linearly blend to the pilot's vertical throttle demand over the
    // following Q_TILT_BTDLY_MS
    float get_back_trans_throttle(uint32_t now, float pilot_throttle_pct);

    // hold the Q_TILT_THR_FT throttle steady for Q_TILT_FTHLD_MS, then
    // linearly blend to the FBWA/commanded throttle over the following
    // Q_TILT_FTBLD_MS
    float get_fwd_trans_throttle(uint32_t now, float commanded_throttle_pct);

    // runs the FW yaw controller during the BACK_HOLD window when
    // Q_TILT_FWHLD_EN is set, otherwise leaves the rudder as the active
    // Q-mode set it
    void update_yaw_authority() const;

    Stage stage = Stage::VTOL;

    // start of each transition sequence, zero when that sequence is not
    // running. Elapsed time against these and Q_TILT_FTHLD_MS/FTBLD_MS
    // forwards, Q_TILT_FWHLD_MS/BTDLY_MS back, gives HOLD, BLEND or done
    uint32_t fwd_trans_start_ms = 0;
    uint32_t back_trans_start_ms = 0;

    // last time fixed wing outputs were driven, zero if never. A back
    // transition starts only while this is within FW_OUTPUT_TIMEOUT_MS, so
    // entering a VTOL mode having never flown forward is not a transition
    uint32_t last_fw_output_ms = 0;
    static const uint32_t FW_OUTPUT_TIMEOUT_MS = 2000;

    // pitch envelope state for set_VTOL_roll_pitch_limit()
    uint32_t last_fw_mode_ms = 0;
    int32_t last_fw_nav_pitch_cd = 0;

    // state from the last throttle blend, read by Tiltrotor::write_log()
    // for the TILT log
    uint32_t backtrans_elapsed_ms = 0;
    float backtrans_pilot_throttle = 0;
    float backtrans_blend_throttle = 0;
    uint32_t fwdtrans_elapsed_ms = 0;
    float fwdtrans_commanded_throttle = 0;
    float fwdtrans_blend_throttle = 0;

    Tiltrotor& tiltrotor;

};
