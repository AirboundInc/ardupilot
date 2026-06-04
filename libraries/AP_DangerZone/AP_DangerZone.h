#pragma once

#include "AP_DangerZone_config.h"

#if AP_DANGERZONE_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>

// Vehicle-agnostic implementation of the Danger Zone framework
// The zone definitions, metric getters and the actions all live in the vehicle code (e.g. ArduPlane/danger_zone.cpp).
// This library only evaluates the registered checks and tracks the current zone level.

class AP_DangerZone {
public:
    AP_DangerZone() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_DangerZone);

    // Current danger zone level. 0 is the baseline (nominal) zone.
    uint8_t get_current_danger_zone() const { return _zone; }

private:
    uint8_t _zone{0};   // current zone level; 0 = baseline
};

#endif  // AP_DANGERZONE_ENABLED
