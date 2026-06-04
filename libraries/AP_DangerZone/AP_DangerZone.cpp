#include "AP_DangerZone.h"

#if AP_DANGERZONE_ENABLED


void AP_DangerZone::init(const DZ_Zone *zones, uint8_t num_zones, DZ_CheckState *states)
{
    _zones = zones;
    _num_zones = num_zones;
    _states = states;
    _zone = 0;
    _reason = "";
    _last_transition_ms = 0;
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

    // Escalation takes priority: evaluate the entry conditions of the next zone.
    // The next zone's entry checks are fed every tick while we sit in the
    // current zone, so their windowed buffers stay warm.
    if (_zone + 1 < _num_zones) {
        DZ_CheckState *entry_states = states_for(_zone + 1, false);
        if (_zones[_zone + 1].entry.update(entry_states, now_ms)) {
            _zone++;
            _reason = _zones[_zone].name;
            _last_transition_ms = now_ms;
            return;                 // at most one transition per tick
        }
    }

    // De-escalation: evaluate the exit conditions of the current zone. Exit
    // debouncing lives in the conditions themselves (e.g. a rolling Window),
    // so de-escalation is immediate once the exit group is satisfied. An empty
    // exit group never fires, so the zone is terminal.
    if (_zone > 0) {
        DZ_CheckState *exit_states = states_for(_zone, true);
        if (_zones[_zone].exit.update(exit_states, now_ms)) {
            _zone--;
            _reason = _zones[_zone].name;
            _last_transition_ms = now_ms;
        }
    }
}

#endif  // AP_DANGERZONE_ENABLED
