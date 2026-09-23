#pragma once

#include <AP_DangerZone/AP_DangerZone_config.h>

#if AP_DANGERZONE_ENABLED

#include "DZ_Check.h"

// A single danger zone level.
//
//   entry: conditions that escalate INTO this zone from the level below.
//   exit:  conditions that de-escalate OUT of this zone to the level below.
//          An empty exit group makes the zone terminal (never de-escalates).
//
// De-escalation requires ALL of: the exit conditions hold, this zone's own
// entry conditions have cleared, and the DZ_HYST_TIMER dwell has elapsed since
// this zone was entered. Escalation is never gated by either mechanism.
struct DZ_Zone {
    const char* name;
    DZ_Group    entry;
    DZ_Group    exit;
};

// DZ_CheckState slots required per zone: one block for entry checks and one for
// exit checks. The vehicle declares an array of
// num_zones * DZ_ZONE_STATE_SLOTS states alongside its zone table.
static const uint8_t DZ_ZONE_STATE_SLOTS = 2 * DZ_MAX_CHECKS;

namespace dz {
    // Total windowed checks across all zones' entry and exit groups.
    // Sizes the DZ_RingBuffer pool the vehicle passes to AP_DangerZone::init().
    constexpr uint16_t buffer_pool_size(const DZ_Zone *zones, uint8_t n, uint8_t i = 0) {
        return i >= n ? 0
            : count_buffers(zones[i].entry) + count_buffers(zones[i].exit)
              + buffer_pool_size(zones, n, i + 1);
    }
}

#endif  // AP_DANGERZONE_ENABLED
