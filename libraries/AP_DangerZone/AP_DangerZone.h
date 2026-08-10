#pragma once

#include "AP_DangerZone_config.h"

#if AP_DANGERZONE_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "DZ_Zone.h"

// Vehicle-agnostic implementation of the Danger Zone framework
// The zone definitions, metric getters and the actions all live in the vehicle code (e.g. ArduPlane/danger_zone_config.cpp).
// This library only evaluates the registered checks and tracks the current zone level.

// Operating mode selected by the DZ_ENABLE parameter.
enum class DZ_Enable : uint8_t {
    OFF        = 0,   // Danger Zone module disabled
    LOG        = 1,   // Run checks and logging, but take no actions
    FULL       = 2,   // Run checks and logging, and take zone actions
};

class AP_DangerZone {
public:
    AP_DangerZone() { AP_Param::setup_object_defaults(this, var_info); }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_DangerZone);

    static const struct AP_Param::GroupInfo var_info[];

    // Operating mode (DZ_ENABLE). Zero means the module is disabled.
    DZ_Enable enabled() const { return DZ_Enable(_enable.get()); }

    // True only in FULL mode, i.e. when zone actions should be taken. Consumers
    // of get_current_danger_zone() that actuate must gate on this so LOG mode
    // tracks and logs without changing behaviour.
    bool actions_enabled() const { return enabled() == DZ_Enable::FULL; }

    // Register the vehicle-owned zone table and the parallel state storage.
    // `states` must hold num_zones * DZ_ZONE_STATE_SLOTS entries.
    // `buffers` is the buffer pool for windowed checks
    void init(const DZ_Zone *zones, uint8_t num_zones, DZ_CheckState *states,
              DZ_RingBuffer *buffers = nullptr, uint8_t num_buffers = 0);

    // Evaluate entry of the next zone, and exit + entry of the current zone.
    // Advances by at most one level per call.
    // De-escalation requires the current zone's own entry must have cleared and 
    // the zone's dwell timer should have elapsed; escalation is always immediate.
    void update(uint32_t now_ms);

    // Return to the baseline zone and clear all check state (debounce timers and
    // rolling buffers), so evaluation starts fresh. Used e.g. on disarm.
    void reset();

    // Clear every check's debounce timer and rolling buffer
    // while staying in the current zone.
    void reset_checks();

    // Current danger zone level
    uint8_t get_current_danger_zone() const { return _zone + 1; }

    // Name of the zone entered/exited at the most recent transition.
    const char *get_reason() const { return _reason; }

    // Timestamp (ms) of the most recent zone transition.
    uint32_t get_last_transition_ms() const { return _last_transition_ms; }

    // Bitmasks for every entry/exit condition
    uint8_t get_entry_bits() const { return _entry_bits; }
    uint8_t get_exit_bits() const { return _exit_bits; }

    // Bitmask of the current zone's own entry checks satisfied this tick
    uint8_t get_self_entry_bits() const { return _self_bits; }

private:
    // Base of the entry (is_exit=false) or exit (is_exit=true) check-state block
    // for a given zone.
    DZ_CheckState *states_for(uint8_t zone, bool is_exit) const;

    // Use a buffer for each windowed check in a group
    void wire_group_buffers(const DZ_Group &group, DZ_CheckState *states,
                            DZ_RingBuffer *buffers, uint8_t num_buffers, uint8_t &next) const;

    // Clear the check states that become active for a zone
    void reset_active_checks(uint8_t zone);

    const DZ_Zone *_zones{nullptr};
    DZ_CheckState *_states{nullptr};
    uint8_t  _num_zones{0};
    uint8_t  _zone{0};                  // current zone index
    const char *_reason{""};            // last transition cause
    uint32_t _last_transition_ms{0};
    uint8_t  _entry_bits{0};            // next-zone entry checks satisfied this tick
    uint8_t  _exit_bits{0};             // current-zone exit checks satisfied this tick
    uint8_t  _self_bits{0};             // current-zone own-entry checks satisfied this tick

    AP_Int8  _enable;                   // DZ_ENABLE operating mode
    AP_Int32 _hyst_timer;               // DZ_HYST_TIMER de-escalation hysteresis timer (ms)
};

#endif  // AP_DANGERZONE_ENABLED
