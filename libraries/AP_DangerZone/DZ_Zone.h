#pragma once

#include <AP_DangerZone/AP_DangerZone_config.h>

#if AP_DANGERZONE_ENABLED

#include "DZ_Check.h"

// A single danger zone level.
//
//   entry: conditions that escalate INTO this zone from the level below.
//   exit:  conditions that de-escalate OUT of this zone to the level below.
//          An empty exit group makes the zone terminal (never de-escalates).
//   hysteresis_ms: the exit condition must hold continuously for this long
//          before the zone actually de-escalates.
struct DZ_Zone {
    const char* name;
    DZ_Group    entry;
    DZ_Group    exit;
    uint32_t    hysteresis_ms;
};

// DZ_CheckState slots required per zone: one block for entry checks and one for
// exit checks. The vehicle declares an array of
// num_zones * DZ_ZONE_STATE_SLOTS states alongside its zone table.
static const uint8_t DZ_ZONE_STATE_SLOTS = 2 * DZ_MAX_CHECKS;

#endif  // AP_DANGERZONE_ENABLED
