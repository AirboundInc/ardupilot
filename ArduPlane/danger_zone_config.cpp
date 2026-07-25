#include "Plane.h"

#if HAL_QUADPLANE_ENABLED && AP_DANGERZONE_ENABLED

#include <AP_DangerZone/DZ_Zone.h>

/*
  Danger Zone vehicle wiring
*/

// ---------------------------------------------------------------------------
// Metric getters
// ---------------------------------------------------------------------------
class DZ_Metrics {
public:
    // pitch rate control effort: abs(P + D + FF)
    static float control_effort()
    {
        const auto *atc = plane.quadplane.attitude_control;
        if (atc == nullptr) {
            return 0.0f;
        }
        const AP_PIDInfo &pid = atc->get_rate_pitch_pid().get_pid_info();
        return fabsf(pid.P + pid.D + pid.FF);
    }

    // abs(pitch error) (degrees) in the AHRS frame.
    static float pitch_error_deg()
    {
        const auto *atc = plane.quadplane.attitude_control;
        const auto *view = plane.quadplane.ahrs_view;
        if (atc == nullptr || view == nullptr) {
            return 0.0f;
        }
        const float target_cd = atc->get_att_target_euler_cd().y;
        return fabsf(target_cd - view->pitch_sensor) * 0.01f;
    }

    // target pitch (degrees)
    static float des_pitch_deg()
    {
        const auto *atc = plane.quadplane.attitude_control;
        if (atc == nullptr) {
            return 0.0f;
        }
        return atc->get_att_target_euler_cd().y * 0.01f;
    }

    // abs(pitch) (degrees) in the AHRS frame
    static float att_pitch_deg()
    {
        const auto *view = plane.quadplane.ahrs_view;
        if (view == nullptr) {
            return 0.0f;
        }
        return fabsf(degrees(view->pitch));
    }

    // EKF pitch
    static float raw_pitch_deg()
    {
        return degrees(AP::ahrs().get_pitch());
    }

    // Same-direction tilt vectoring saturation (DesL/DesR), in centidegrees
    // Returns the lesser-magnitude of the left/right tilt outputs when both share a
    // sign, else 0, so abs(value) > thresh means BOTH tilts are maxed the same way.
    static float tilt_same_dir()
    {
        const float l = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        const float r = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        if ((l >= 0) != (r >= 0)) {
            return 0.0f;
        }
        return (fabsf(l) < fabsf(r)) ? l : r;
    }

    // ---- Getters for parameter values ----
    static float ab_pit_lim()   { return plane.autobailout.pit_lim; }
    static float ab_pit_tout()  { return plane.autobailout.pit_tout; }
    static float ab_para_ang()  { return plane.autobailout.para_ang; }
    static float ab_para_tout() { return plane.autobailout.para_tout; }
    static float ab_avg_lim()   { return plane.autobailout.avg_lim; }
    static float ab_peak_lim()  { return plane.autobailout.peak_lim; }
    // recovery window (s -> ms), clamped to the buffer's max retained window
    static float ab_win_ms()
    {
        const float s = constrain_float(plane.autobailout.win_tim, 0.1f, DZ_BUFFER_MAX_WINDOW_MS * 0.001f);
        return s * 1000.0f;
    }
};

// ---------------------------------------------------------------------------
// Autobailout parameters
// ---------------------------------------------------------------------------
const AP_Param::GroupInfo Autobailout::var_info[] = {
    // @Param: PIT_LIM
    // @DisplayName: Autobailout pitch limit
    // @Description: AHRS pitch angle below which autobailout (switch to QLOITER) is triggered.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PIT_LIM", 1, Autobailout, pit_lim, 40),

    // @Param: PIT_TOUT
    // @DisplayName: Autobailout pitch timeout
    // @Description: Time the pitch must remain below AB_PIT_LIM before autobailout triggers.
    // @Units: ms
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("PIT_TOUT", 2, Autobailout, pit_tout, 200),

    // @Param: BTRN_DLY
    // @DisplayName: Autobailout back-transition delay
    // @Description: Delay after the back-transition to hover completes before autobailout may arm, letting attitude settle.
    // @Units: ms
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("BTRN_DLY", 3, Autobailout, btrn_dly, 2500),

    // @Param: PARA_ANG
    // @DisplayName: Autobailout parachute pitch angle
    // @Description: AHRS pitch angle below which the parachute deploy action is triggered.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PARA_ANG", 4, Autobailout, para_ang, -15),

    // @Param: PARA_TOUT
    // @DisplayName: Autobailout parachute timeout
    // @Description: Time the pitch must remain below AB_PARA_ANG before the parachute action triggers.
    // @Units: ms
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("PARA_TOUT", 5, Autobailout, para_tout, 100),

    // @Param: PARA_EN
    // @DisplayName: Autobailout parachute enable
    // @Description: Enable the Zone 5 parachute deploy + disarm action.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PARA_EN", 6, Autobailout, para_en, 1),

    // @Param: WIN_TIM
    // @DisplayName: Autobailout recovery window
    // @Description: Rolling window over which the recovery statistics are evaluated to leave the autobailout zone.
    // @Units: s
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("WIN_TIM", 7, Autobailout, win_tim, 5),

    // @Param: AVG_LIM
    // @DisplayName: Autobailout recovery mean pitch-error limit
    // @Description: Mean pitch error over the recovery window must fall below this to leave the autobailout zone.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("AVG_LIM", 8, Autobailout, avg_lim, 20),

    // @Param: PEAK_LIM
    // @DisplayName: Autobailout recovery peak pitch limit
    // @Description: Peak pitch over the recovery window must fall below this to leave the autobailout zone.
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PEAK_LIM", 9, Autobailout, peak_lim, 30),

    AP_GROUPEND
};

// ---------------------------------------------------------------------------
// Zone table
// ---------------------------------------------------------------------------
static constexpr DZ_Zone dz_zones[] = {
    { .name = "zone1" },   // baseline: nominal vertical flight

    // Zone 2: Weathervaning disabled
    // entry: control effort > 0.1
    // exit:  rolling-avg pitch error < 5 deg AND peak < 10 deg over 5 s
    //        AND control effort < 0.08
    { .name = "zone2",
      .entry = dz::OR(
          DZ_Check::Threshold(DZ_Metrics::control_effort,
              { .thresh = 0.1f, .cmp = DZ_Cmp::ABOVE })),
      .exit = dz::AND(
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::MEAN, .thresh = 5.0f,  .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::PEAK, .thresh = 10.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Threshold(DZ_Metrics::control_effort,
              { .thresh = 0.08f, .cmp = DZ_Cmp::BELOW })) },

    // Zone 3: relax attitude + disable yaw rate
    // entry: pitch error > 20 deg
    //        OR DesPitch oscillation range > 10 deg
    //        OR |att pitch| > 45 deg
    // exit:  rolling-avg pitch error < 10 deg AND peak < 15 deg over 5 s
    { .name = "zone3",
      .entry = dz::OR(
          DZ_Check::Threshold(DZ_Metrics::pitch_error_deg,
              { .thresh = 20.0f, .cmp = DZ_Cmp::ABOVE }),
          DZ_Check::Window(DZ_Metrics::des_pitch_deg,
              { .stat = DZ_Stat::RANGE, .thresh = 10.0f, .cmp = DZ_Cmp::ABOVE, .window_ms = 2000, .duration_ms = 0 }),
          DZ_Check::Threshold(DZ_Metrics::att_pitch_deg,
              { .thresh = 45.0f, .cmp = DZ_Cmp::ABOVE })),
      .exit = dz::AND(
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::MEAN, .thresh = 10.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::PEAK, .thresh = 15.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 })) },

    // Zone 4: autobailout
    // entry: raw pitch < 40 deg for 200 ms
    //        OR both tilts maxed same direction (> 4499 or < -4499 cd, i.e. pinned
    //        to the +/-4500 rail; strict > can't match 4500 exactly) for 100 ms
    // exit:  rolling-avg pitch error < 20 deg AND peak |att pitch| < 30 deg over 5 s
    { .name = "zone4",
      .entry = dz::OR(
          DZ_Check::Duration(DZ_Metrics::raw_pitch_deg,
              { .thresh = 40.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 200,
                .thresh_fn = DZ_Metrics::ab_pit_lim, .dur_fn = DZ_Metrics::ab_pit_tout }),
          DZ_Check::Duration(DZ_Metrics::tilt_same_dir,
              { .thresh = 4499.0f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 100 }),
          DZ_Check::Duration(DZ_Metrics::tilt_same_dir,
              { .thresh = -4499.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 100 })),
      .exit = dz::AND(
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::MEAN, .thresh = 20.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0,
                .thresh_fn = DZ_Metrics::ab_avg_lim, .win_fn = DZ_Metrics::ab_win_ms }),
          DZ_Check::Window(DZ_Metrics::att_pitch_deg,
              { .stat = DZ_Stat::PEAK, .thresh = 30.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0,
                .thresh_fn = DZ_Metrics::ab_peak_lim, .win_fn = DZ_Metrics::ab_win_ms })) },

    // Zone 5: deploy parachute + disarm
    // entry: raw pitch < -15 deg for 100 ms
    //        OR TVs maxed same direction (> 4499 or < -4499 cd, i.e. pinned to the
    //        +/-4500 rail; strict > can't match 4500 exactly) for 500 ms
    // exit:  none
    { .name = "zone5",
      .entry = dz::OR(
          DZ_Check::Duration(DZ_Metrics::raw_pitch_deg,
              { .thresh = -15.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 100,
                .thresh_fn = DZ_Metrics::ab_para_ang, .dur_fn = DZ_Metrics::ab_para_tout }),
          DZ_Check::Duration(DZ_Metrics::tilt_same_dir,
              { .thresh = 4499.0f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 500 }),
          DZ_Check::Duration(DZ_Metrics::tilt_same_dir,
              { .thresh = -4499.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 500 })) },
};

// Parallel check-state storage (debounce timers), sized from the table.
static DZ_CheckState dz_states[ARRAY_SIZE(dz_zones) * DZ_ZONE_STATE_SLOTS];

// Rolling-buffer pool, sized to the windowed checks in the table.
static DZ_RingBuffer dz_buffers[dz::buffer_pool_size(dz_zones, ARRAY_SIZE(dz_zones))];

void Plane::danger_zone_init()
{
    danger_zone.init(dz_zones, ARRAY_SIZE(dz_zones), dz_states, dz_buffers, ARRAY_SIZE(dz_buffers));
    danger_zone_last_level = danger_zone.get_current_danger_zone();
}

// 50Hz scheduler task
void Plane::danger_zone_update()
{
    // only run when armed and in the VTOL phase
    if (danger_zone.enabled() == DZ_Enable::OFF ||
        !arming.is_armed() || !quadplane.in_vtol_mode()) {
        danger_zone.reset();
        danger_zone_last_level = danger_zone.get_current_danger_zone();
        danger_zone_vtol_entry_ms = 0;
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // Store the VTOL settle time after the backtransition delay
    if (danger_zone_vtol_entry_ms == 0 || quadplane.tailsitter_in_vtol_transition()) {
        danger_zone_vtol_entry_ms = now;
    }
    const bool backtransition_done = (now - danger_zone_vtol_entry_ms) >= (uint32_t)autobailout.btrn_dly;

    danger_zone.update(now);

    const uint8_t level = danger_zone.get_current_danger_zone();

#if HAL_LOGGING_ENABLED
    // Log the current zone level and bitmasks of the evaluations of 
    // the entry and exit conditions for each zone
    AP::logger().WriteStreaming("DZ", "TimeUS,Zone,Ent,Ext,SEnt", "QBBBB",
                                AP_HAL::micros64(),
                                level,
                                danger_zone.get_entry_bits(),
                                danger_zone.get_exit_bits(),
                                danger_zone.get_self_entry_bits());
#endif

    // Handle autobailout
    if (danger_zone.actions_enabled() && level >= 4 && backtransition_done) {
        const bool in_bailout = control_mode == &mode_qloiter &&
                                control_mode_reason == ModeReason::DANGERZONE_BAILOUT;
        const Mode::Number mode_num = control_mode->mode_number();
        const bool bailout_excluded =
            mode_num == Mode::Number::QSTABILIZE ||
            mode_num == Mode::Number::QHOVER ||
            mode_num == Mode::Number::QLOITER ||
#if QAUTOTUNE_ENABLED
            mode_num == Mode::Number::QAUTOTUNE ||
#endif
            mode_num == Mode::Number::QACRO;
        if (!in_bailout && !bailout_excluded) {
            // save the pre-bailout mode so it can be restored on recovery
            danger_zone_resume_mode = mode_num;
            set_mode_by_number(Mode::Number::QLOITER, ModeReason::DANGERZONE_BAILOUT);
        }
    }

    // Zone transition
    if (level != danger_zone_last_level) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DangerZone: %s (level %u)",
                        danger_zone.get_reason(), (unsigned)level);

        // Only run actions in Full mode
        if (danger_zone.actions_enabled()) {
            // Mission resumption after exiting Zone 4
            // Only resume if the current mode is QLoiter 
            // and the mode reason is bailout from the Danger Zone module
            if (level < 4 && danger_zone_last_level >= 4 &&
                control_mode == &mode_qloiter &&
                control_mode_reason == ModeReason::DANGERZONE_BAILOUT) {
                set_mode_by_number(danger_zone_resume_mode, ModeReason::DANGERZONE_RECOVERED);
            }

            // Zone 5: Disarm and deploy parachute (if enabled)
            if (level >= 5 && danger_zone_last_level < 5 && autobailout.para_en) {
#if PARACHUTE == ENABLED
                parachute_release_with_disarm();
#endif
            }
        }

        danger_zone_last_level = level;
    }
}

#endif  // HAL_QUADPLANE_ENABLED && AP_DANGERZONE_ENABLED
