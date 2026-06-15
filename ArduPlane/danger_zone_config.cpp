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
    // exit:  rolling-avg pitch error < 20 deg AND peak < 30 deg over 5 s
    { .name = "zone4",
      .entry = dz::OR(
          DZ_Check::Duration(DZ_Metrics::raw_pitch_deg,
              { .thresh = 40.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 200 }),
          DZ_Check::Duration(DZ_Metrics::tilt_same_dir,
              { .thresh = 4499.0f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 100 }),
          DZ_Check::Duration(DZ_Metrics::tilt_same_dir,
              { .thresh = -4499.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 100 })),
      .exit = dz::AND(
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::MEAN, .thresh = 20.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::PEAK, .thresh = 30.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 })) },

    // Zone 5: deploy parachute + disarm
    // entry: raw pitch < -15 deg for 100 ms
    //        OR TVs maxed same direction (> 4499 or < -4499 cd, i.e. pinned to the
    //        +/-4500 rail; strict > can't match 4500 exactly) for 500 ms
    // exit:  none
    { .name = "zone5",
      .entry = dz::OR(
          DZ_Check::Duration(DZ_Metrics::raw_pitch_deg,
              { .thresh = -15.0f, .cmp = DZ_Cmp::BELOW, .duration_ms = 100 }),
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
    if (!arming.is_armed() || !quadplane.in_vtol_mode()) {
        danger_zone.reset();
        danger_zone_last_level = danger_zone.get_current_danger_zone();
        return;
    }

    danger_zone.update(AP_HAL::millis());

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

    // Zone actions
    if (level != danger_zone_last_level) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DangerZone: %s (level %u)",
                        danger_zone.get_reason(), (unsigned)level);

        // Zone 4: autobailout to QLOITER
        if (level >= 4 && danger_zone_last_level < 4) {
            set_mode_by_number(Mode::Number::QLOITER, ModeReason::DANGERZONE_BAILOUT);
        }

        // // Mission resumption after exiting Zone 4
        // if (level < 4 && danger_zone_last_level >= 4) {
        // }

        // Zone 5: Disarm and deploy parachute
        if (level >= 5 && danger_zone_last_level < 5) {
#if PARACHUTE == ENABLED
            parachute_release_with_disarm();
#endif
        }

        danger_zone_last_level = level;
    }
}

#endif  // HAL_QUADPLANE_ENABLED && AP_DANGERZONE_ENABLED
