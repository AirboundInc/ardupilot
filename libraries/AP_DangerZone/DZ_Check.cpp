#include "DZ_Check.h"

#if AP_DANGERZONE_ENABLED

// ---------------------------------------------------------------------------
// DZ_RingBuffer
// ---------------------------------------------------------------------------
void DZ_RingBuffer::push(float v, uint32_t now_ms)
{
    if (!_has_first) {
        _first_ms = now_ms;
        _has_first = true;
    }
    const uint16_t w = (_tail + _count) % DZ_CHECK_BUFFER_SAMPLES;
    _v[w] = v;
    _t[w] = now_ms;
    if (_count < DZ_CHECK_BUFFER_SAMPLES) {
        _count++;
    } else {
        // buffer full: overwrite oldest by advancing the tail
        _tail = (_tail + 1) % DZ_CHECK_BUFFER_SAMPLES;
    }
}

void DZ_RingBuffer::prune(uint32_t now_ms, uint32_t window_ms)
{
    // drop samples older than the window (timestamps assumed monotonic)
    while (_count > 0 && (now_ms - _t[_tail]) > window_ms) {
        _tail = (_tail + 1) % DZ_CHECK_BUFFER_SAMPLES;
        _count--;
    }
}

float DZ_RingBuffer::mean() const
{
    if (_count == 0) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (uint16_t k = 0; k < _count; k++) {
        sum += _v[(_tail + k) % DZ_CHECK_BUFFER_SAMPLES];
    }
    return sum / _count;
}

float DZ_RingBuffer::peak() const
{
    if (_count == 0) {
        return 0.0f;
    }
    float mx = _v[_tail];
    for (uint16_t k = 1; k < _count; k++) {
        const float x = _v[(_tail + k) % DZ_CHECK_BUFFER_SAMPLES];
        if (x > mx) { mx = x; }
    }
    return mx;
}

float DZ_RingBuffer::range() const
{
    if (_count == 0) {
        return 0.0f;
    }
    float mn = _v[_tail];
    float mx = _v[_tail];
    for (uint16_t k = 1; k < _count; k++) {
        const float x = _v[(_tail + k) % DZ_CHECK_BUFFER_SAMPLES];
        if (x < mn) { mn = x; }
        if (x > mx) { mx = x; }
    }
    return mx - mn;
}

uint16_t DZ_RingBuffer::mean_crossings() const
{
    if (_count < 2) {
        return 0;
    }
    const float m = mean();
    uint16_t crossings = 0;
    bool prev_pos = (_v[_tail] - m) >= 0.0f;
    for (uint16_t k = 1; k < _count; k++) {
        const float d = _v[(_tail + k) % DZ_CHECK_BUFFER_SAMPLES] - m;
        const bool pos = d >= 0.0f;
        if (pos != prev_pos) {
            crossings++;
            prev_pos = pos;
        }
    }
    return crossings;
}

// ---------------------------------------------------------------------------
// DZ_Check
// ---------------------------------------------------------------------------
static bool cmp_pass(float v, float thresh, DZ_Cmp cmp)
{
    return (cmp == DZ_Cmp::ABOVE) ? (v > thresh) : (v < thresh);
}

bool DZ_Check::raw_satisfied(DZ_CheckState& st, float value, uint32_t now_ms) const
{
    switch (type) {
    case DZ_CheckType::THRESHOLD:
    case DZ_CheckType::DURATION:
        return cmp_pass(value, thresh, cmp);

    case DZ_CheckType::WINDOW: {
        st.buf.push(value, now_ms);
        st.buf.prune(now_ms, window_ms);
        // Require a complete window before the statistic is meaningful, so a
        // partially-filled buffer cannot satisfy the check prematurely.
        if (!st.buf.full(now_ms, window_ms)) {
            return false;
        }
        float s = 0.0f;
        switch (stat) {
        case DZ_Stat::MEAN:  s = st.buf.mean();  break;
        case DZ_Stat::PEAK:  s = st.buf.peak();  break;
        case DZ_Stat::RANGE: s = st.buf.range(); break;
        }
        return cmp_pass(s, thresh, cmp);
    }

    case DZ_CheckType::OSCILLATION: {
        st.buf.push(value, now_ms);
        st.buf.prune(now_ms, window_ms);
        if (!st.buf.full(now_ms, window_ms)) {
            return false;
        }
        return st.buf.mean_crossings() >= min_crossings;
    }

    case DZ_CheckType::NONE:
    default:
        return false;
    }
}

bool DZ_Check::update(DZ_CheckState& st, float value, uint32_t now_ms) const
{
    const bool raw = raw_satisfied(st, value, now_ms);
    if (!raw) {
        st.raw_active = false;
        return false;
    }
    if (duration_ms == 0) {
        return true;
    }
    if (!st.raw_active) {
        st.raw_active = true;
        st.raw_since_ms = now_ms;
    }
    return (now_ms - st.raw_since_ms) >= duration_ms;
}

// ---------------------------------------------------------------------------
// DZ_Group
// ---------------------------------------------------------------------------
bool DZ_Group::update(DZ_CheckState states[DZ_MAX_CHECKS], uint32_t now_ms,
                      uint8_t *out_mask) const
{
    bool any = false;
    bool all = true;
    bool saw_check = false;
    uint8_t mask = 0;

    for (uint8_t i = 0; i < DZ_MAX_CHECKS; i++) {
        const DZ_Check& c = checks[i];
        if (c.type == DZ_CheckType::NONE) {
            continue;
        }
        saw_check = true;
        // All checks are updated every tick so their buffers/timers advance;
        // we never short-circuit the side-effecting update.
        const float value = (c.getter != nullptr) ? c.getter() : 0.0f;
        const bool sat = c.update(states[i], value, now_ms);
        if (sat) {
            mask |= (1U << i);
        }
        any = any || sat;
        all = all && sat;
    }

    if (out_mask != nullptr) {
        *out_mask = mask;
    }
    if (!saw_check) {
        return false;   // empty group is never satisfied
    }
    return (op == DZ_Op::OR) ? any : all;
}

#endif  // AP_DANGERZONE_ENABLED
