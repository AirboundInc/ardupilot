#include "AP_DangerZone.h"

#if AP_DANGERZONE_ENABLED

const AP_Param::GroupInfo AP_DangerZone::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Danger Zone enable
    // @Description: Enables the Danger Zone module. In Log mode the zone checks run and are logged but no actions are taken; in Enabled mode the zone actions are also executed.
    // @Values: 0:Disabled,1:Log (checks and logging only),2:Enabled (checks, logging and actions)
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_DangerZone, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HYST_TIMER
    // @DisplayName: Danger Zone hysteresis timer
    // @Description: Minimum time to remain in a zone before de-escalating to the previous zone. Applied to every zone to suppress flicker; a zone may specify a larger minimum of its own.
    // @Units: ms
    // @Range: 0 30000
    // @User: Standard
    AP_GROUPINFO("HYST_TIMER", 2, AP_DangerZone, _hyst_timer, 0),

    AP_GROUPEND
};

void AP_DangerZone::init(const DZ_Zone *zones, uint8_t num_zones, DZ_CheckState *states,
                         DZ_RingBuffer *buffers, uint8_t num_buffers)
{
    _zones = zones;
    _num_zones = num_zones;
    _states = states;

    // Use a buffer for each windowed check
    uint8_t next = 0;
    for (uint8_t z = 0; z < num_zones; z++) {
        wire_group_buffers(zones[z].entry, states_for(z, false), buffers, num_buffers, next);
        wire_group_buffers(zones[z].exit,  states_for(z, true),  buffers, num_buffers, next);
    }

    reset();
}

void AP_DangerZone::reset()
{
    _zone = 0;
    _reason = "";
    _last_transition_ms = 0;
    _entry_bits = 0;
    _exit_bits = 0;
    _self_bits = 0;

    // Clear each check's debounce timer and rolling buffer (buffer wiring is
    // preserved; only the accumulated data is dropped).
    if (_states != nullptr) {
        for (uint16_t i = 0; i < (uint16_t)_num_zones * DZ_ZONE_STATE_SLOTS; i++) {
            _states[i].reset();
        }
    }
}

void AP_DangerZone::wire_group_buffers(const DZ_Group &group, DZ_CheckState *states,
                                       DZ_RingBuffer *buffers, uint8_t num_buffers,
                                       uint8_t &next) const
{
    for (uint8_t i = 0; i < DZ_MAX_CHECKS; i++) {
        // Only use a buffer for checks which need one
        if (!dz::needs_buffer(group.checks[i].type)) {
            continue;
        }
        if (next < num_buffers) {
            states[i].buf = &buffers[next++];
            states[i].buf->reset();
        }
    }
}

DZ_CheckState *AP_DangerZone::states_for(uint8_t zone, bool is_exit) const
{
    return &_states[zone * DZ_ZONE_STATE_SLOTS + (is_exit ? DZ_MAX_CHECKS : 0)];
}

void AP_DangerZone::reset_active_checks(uint8_t zone)
{
    if (_states == nullptr) {
        return;
    }
    DZ_CheckState *ex = states_for(zone, true);    // current zone exit
    DZ_CheckState *en = states_for(zone, false);   // current zone own-entry
    for (uint8_t i = 0; i < DZ_MAX_CHECKS; i++) {
        ex[i].reset();
        en[i].reset();
    }
    if (zone + 1 < _num_zones) {
        DZ_CheckState *nx = states_for(zone + 1, false);   // next zone entry
        for (uint8_t i = 0; i < DZ_MAX_CHECKS; i++) {
            nx[i].reset();
        }
    }
}

void AP_DangerZone::update(uint32_t now_ms)
{
    if (_zones == nullptr || _states == nullptr || _num_zones == 0) {
        return;
    }

    _entry_bits = 0;
    _exit_bits = 0;
    _self_bits = 0;
    bool want_escalate = false;
    bool want_deescalate = false;
    bool self_entry = false;

    // Check the entry conditions for the next zone
    if (_zone + 1 < _num_zones) {
        DZ_CheckState *entry_states = states_for(_zone + 1, false);
        want_escalate = _zones[_zone + 1].entry.update(entry_states, now_ms, &_entry_bits);
    }

    if (_zone > 0) {
        // Check the exit conditions for the current zone
        DZ_CheckState *exit_states = states_for(_zone, true);
        want_deescalate = _zones[_zone].exit.update(exit_states, now_ms, &_exit_bits);

        // Re-evaluate the current zone's own entry conditions
        DZ_CheckState *self_states = states_for(_zone, false);
        self_entry = _zones[_zone].entry.update(self_states, now_ms, &_self_bits);
    }

    if (want_escalate) {
        // Entry condition triggered, enter the next zone
        _zone++;
        _reason = _zones[_zone].name;
        _last_transition_ms = now_ms;
        reset_active_checks(_zone);
    } else {
        // Use the DZ_HYST_TIMER value for de-escalation
        const uint32_t hyst_ms = _hyst_timer > 0 ? (uint32_t)_hyst_timer.get() : 0;
        if (want_deescalate && !self_entry && (now_ms - _last_transition_ms) >= hyst_ms) {
            // For de-escalation, the exit condition should hold, the current zone's entry condition should clear,
            // and the hysteresis timers should have elapsed
            _zone--;
            _reason = _zones[_zone].name;
            _last_transition_ms = now_ms;
            reset_active_checks(_zone);
        }
    }
}

#endif  // AP_DANGERZONE_ENABLED
