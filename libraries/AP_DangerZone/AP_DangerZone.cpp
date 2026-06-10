#include "AP_DangerZone.h"

#if AP_DANGERZONE_ENABLED


void AP_DangerZone::init(const DZ_Zone *zones, uint8_t num_zones, DZ_CheckState *states,
                         DZ_RingBuffer *buffers, uint8_t num_buffers)
{
    _zones = zones;
    _num_zones = num_zones;
    _states = states;
    _zone = 0;
    _reason = "";
    _last_transition_ms = 0;

    // Use a buffer for each windowed check
    uint8_t next = 0;
    for (uint8_t z = 0; z < num_zones; z++) {
        wire_group_buffers(zones[z].entry, states_for(z, false), buffers, num_buffers, next);
        wire_group_buffers(zones[z].exit,  states_for(z, true),  buffers, num_buffers, next);
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

void AP_DangerZone::update(uint32_t now_ms)
{
    if (_zones == nullptr || _states == nullptr || _num_zones == 0) {
        return;
    }

    _entry_bits = 0;
    _exit_bits = 0;
    bool want_escalate = false;
    bool want_deescalate = false;

    // Check the entry conditions for the next zone
    if (_zone + 1 < _num_zones) {
        DZ_CheckState *entry_states = states_for(_zone + 1, false);
        want_escalate = _zones[_zone + 1].entry.update(entry_states, now_ms, &_entry_bits);
    }

    // Check the exit conditions for the current zone
    if (_zone > 0) {
        DZ_CheckState *exit_states = states_for(_zone, true);
        want_deescalate = _zones[_zone].exit.update(exit_states, now_ms, &_exit_bits);
    }

    if (want_escalate) {
        // Entry condition triggered, enter the next zone
        _zone++;
        _reason = _zones[_zone].name;
        _last_transition_ms = now_ms;
    } else if (want_deescalate) {
        // Exit condition triggered, enter the previous zone
        _zone--;
        _reason = _zones[_zone].name;
        _last_transition_ms = now_ms;
    }
}

#endif  // AP_DANGERZONE_ENABLED
