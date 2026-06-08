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
};

// ---------------------------------------------------------------------------
// Zone table
// ---------------------------------------------------------------------------
static const DZ_Zone dz_zones[] = {
    { .name = "zone1" },   // baseline: nominal vertical flight

    // ---- zone2 - Caution: weathervaning disabled --------------------------
    // entry: control effort > 0.1
    // exit:  rolling-avg pitch error < 5 deg AND peak < 10 deg over 5 s
    //        AND control effort < 0.08
    { .name = "zone2",
      .entry = dz::OR(
          DZ_Check::Threshold(DZ_Metrics::control_effort,
              { .thresh = 0.1f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 0 })),
      .exit = dz::AND(
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::MEAN, .thresh = 5.0f,  .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::PEAK, .thresh = 10.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Threshold(DZ_Metrics::control_effort,
              { .thresh = 0.08f, .cmp = DZ_Cmp::BELOW, .duration_ms = 0 })) },

    // ---- zone3 - Warning: relax attitude + disable yaw rate ---------------
    // entry: pitch error > 20 deg
    //        OR DesPitch oscillation range > 10 deg
    //        OR |att pitch| > 45 deg
    // exit:  rolling-avg pitch error < 10 deg AND peak < 15 deg over 5 s
    { .name = "zone3",
      .entry = dz::OR(
          DZ_Check::Threshold(DZ_Metrics::pitch_error_deg,
              { .thresh = 20.0f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 0 }),
          DZ_Check::Window(DZ_Metrics::des_pitch_deg,
              { .stat = DZ_Stat::RANGE, .thresh = 10.0f, .cmp = DZ_Cmp::ABOVE, .window_ms = 2000, .duration_ms = 0 }),
          DZ_Check::Threshold(DZ_Metrics::att_pitch_deg,
              { .thresh = 45.0f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 0 })),
      .exit = dz::AND(
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::MEAN, .thresh = 10.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 }),
          DZ_Check::Window(DZ_Metrics::pitch_error_deg,
              { .stat = DZ_Stat::PEAK, .thresh = 15.0f, .cmp = DZ_Cmp::BELOW, .window_ms = 5000, .duration_ms = 0 })) },
};

// Parallel check-state storage (timers + rolling buffers), sized from the table.
static DZ_CheckState dz_states[ARRAY_SIZE(dz_zones) * DZ_ZONE_STATE_SLOTS];

void Plane::danger_zone_init()
{
    danger_zone.init(dz_zones, ARRAY_SIZE(dz_zones), dz_states);
    danger_zone_last_level = danger_zone.get_current_danger_zone();
}

// 50Hz scheduler task. Runs only in the VTOL phase.
void Plane::danger_zone_update()
{
    if (!quadplane.in_vtol_mode()) {
        return;
    }

    danger_zone.update(AP_HAL::millis());

    const uint8_t level = danger_zone.get_current_danger_zone();

#if HAL_LOGGING_ENABLED
    // Log the current zone level and bitmasks of the evaluations of 
    // the entry and exit conditions for each zone
    AP::logger().WriteStreaming("DZ", "TimeUS,Zone,Ent,Ext", "QBBB",
                                AP_HAL::micros64(),
                                level,
                                danger_zone.get_entry_bits(),
                                danger_zone.get_exit_bits());
#endif

    // Send a warning on a zone change
    if (level != danger_zone_last_level) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DangerZone: %s (level %u)",
                        danger_zone.get_reason(), (unsigned)level);
        danger_zone_last_level = level;
    }
}

#endif  // HAL_QUADPLANE_ENABLED && AP_DANGERZONE_ENABLED
