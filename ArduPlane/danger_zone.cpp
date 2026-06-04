#include "Plane.h"

#if HAL_QUADPLANE_ENABLED && AP_DANGERZONE_ENABLED

#include <AP_DangerZone/DZ_Zone.h>

/*
  Danger Zone vehicle wiring.
*/

// ---------------------------------------------------------------------------
// Metric getters
//
// Each getter returns the current value of one scalar metric and is referenced
// by a check in the zone table. They are plain functions so their address can
// be stored as a DZ_Getter. Example shape:
//
//   static float dz_control_effort()
//   {
//       const auto *atc = plane.quadplane.get_attitude_control();   // needs accessor
//       if (atc == nullptr) {
//           return 0.0f;
//       }
//       const AP_PIDInfo &pid = atc->get_rate_pitch_pid().get_pid_info();
//       return fabsf(pid.P + pid.D + pid.FF);
//   }
//
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Zone table
//
// Zones are ordered by severity; index 0 is the baseline (nominal) zone.
// Example shape for a real zone (to be filled in the definition phase):
//
//   { .name = "zone2",
//     .entry = dz::OR(
//         DZ_Check::Threshold(dz_control_effort, { .thresh = 0.08f, .cmp = DZ_Cmp::ABOVE, .duration_ms = 0 }),
//         DZ_Check::Window(dz_pitch_error, { .stat = DZ_Stat::PEAK, .thresh = 10.0f, .cmp = DZ_Cmp::ABOVE, .window_ms = 5000, .duration_ms = 0 })),
//     .exit  = dz::AND(
//         DZ_Check::Threshold(dz_control_effort, { .thresh = 0.08f, .cmp = DZ_Cmp::BELOW, .duration_ms = 0 })),
//     .hysteresis_ms = 5000 },
// ---------------------------------------------------------------------------
static const DZ_Zone dz_zones[] = {
    { .name = "zone1" },   // baseline only; real zones added in the definition phase
};

// Parallel check-state storage
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

    // Surface transitions to the GCS (proper DataFlash logging arrives in the
    // logging phase).
    const uint8_t level = danger_zone.get_current_danger_zone();
    if (level != danger_zone_last_level) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DangerZone: %s (level %u)",
                        danger_zone.get_reason(), (unsigned)level);
        danger_zone_last_level = level;
    }
}

#endif  // HAL_QUADPLANE_ENABLED && AP_DANGERZONE_ENABLED
