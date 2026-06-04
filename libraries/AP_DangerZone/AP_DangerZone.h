#pragma once

#include "AP_DangerZone_config.h"

#if AP_DANGERZONE_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include "DZ_Zone.h"

// Vehicle-agnostic implementation of the Danger Zone framework
// The zone definitions, metric getters and the actions all live in the vehicle code (e.g. ArduPlane/danger_zone.cpp).
// This library only evaluates the registered checks and tracks the current zone level.

class AP_DangerZone {
public:
    AP_DangerZone() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_DangerZone);

    // Register the vehicle-owned zone table and the parallel state storage.
    // `states` must hold num_zones * DZ_ZONE_STATE_SLOTS entries.
    void init(const DZ_Zone *zones, uint8_t num_zones, DZ_CheckState *states);

    // Evaluate entry of the next zone and exit of the current zone. Advances by
    // at most one level per call; escalation takes priority over de-escalation.
    void update(uint32_t now_ms);

    // Current danger zone level. 0 is the baseline (nominal) zone.
    uint8_t get_current_danger_zone() const { return _zone; }

    // Name of the zone entered/exited at the most recent transition.
    const char *get_reason() const { return _reason; }

    // Timestamp (ms) of the most recent zone transition.
    uint32_t get_last_transition_ms() const { return _last_transition_ms; }

private:
    // Base of the entry (is_exit=false) or exit (is_exit=true) check-state block
    // for a given zone.
    DZ_CheckState *states_for(uint8_t zone, bool is_exit) const;

    const DZ_Zone *_zones{nullptr};
    DZ_CheckState *_states{nullptr};
    uint8_t  _num_zones{0};
    uint8_t  _zone{0};                  // current zone level; 0 = baseline
    const char *_reason{""};            // last transition cause
    uint32_t _last_transition_ms{0};
};

#endif  // AP_DANGERZONE_ENABLED
