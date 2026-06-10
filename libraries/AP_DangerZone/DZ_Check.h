#pragma once

#include <AP_DangerZone/AP_DangerZone_config.h>

#if AP_DANGERZONE_ENABLED

#include <stdint.h>

// ---------------------------------------------------------------------------
// Danger Zone checks
//
// A DZ_Check is a small, homogeneous, value-type description of one predicate
// over a single scalar metric. The four named check types share a flat set of
// fields (unused fields are simply ignored per type) so that checks can live
// inline in a constexpr table and be stored in fixed-capacity arrays.
//
// THRESHOLD fires the instant its predicate holds; it carries no duration.
// DURATION fires only after the predicate holds continuously for duration_ms.
//
// Checks are pure: the live metric value is injected into update() by the
// caller (the state machine reads it via the stored getter). Nothing is read
// inside the check except the value handed in, which keeps the logic trivially
// unit-testable.
// ---------------------------------------------------------------------------

// Maximum window the rolling buffer can retain, and the assumed update rate.
// Together they bound the per-check sample buffer.
#ifndef DZ_UPDATE_RATE_HZ
#define DZ_UPDATE_RATE_HZ 50
#endif
#ifndef DZ_BUFFER_MAX_WINDOW_MS
#define DZ_BUFFER_MAX_WINDOW_MS 10000
#endif

// Number of samples retained per windowed check.
static const uint16_t DZ_CHECK_BUFFER_SAMPLES =
    (DZ_BUFFER_MAX_WINDOW_MS / 1000) * DZ_UPDATE_RATE_HZ;

// Maximum checks combined within a single AND/OR group.
static const uint8_t DZ_MAX_CHECKS = 4;

enum class DZ_CheckType : uint8_t { NONE = 0, THRESHOLD, DURATION, WINDOW, OSCILLATION };
enum class DZ_Cmp  : uint8_t { ABOVE = 0, BELOW };       // ABOVE is the default (value-init)
enum class DZ_Stat : uint8_t { MEAN = 0, PEAK, RANGE };  // statistic reduced over a window
enum class DZ_Op   : uint8_t { OR = 0, AND };

// Live metric source. Returns the current value of one scalar metric.
typedef float (*DZ_Getter)();

// Per-check parameter structs, used for inline named-argument construction.
struct DZ_ThresholdParams { float thresh; DZ_Cmp cmp; };
struct DZ_DurationParams  { float thresh; DZ_Cmp cmp; uint32_t duration_ms; };
struct DZ_WindowParams    { DZ_Stat stat; float thresh; DZ_Cmp cmp; uint32_t window_ms; uint32_t duration_ms; };
struct DZ_OscParams       { uint32_t window_ms; uint16_t min_crossings; uint32_t duration_ms; };

// ---------------------------------------------------------------------------
// Rolling sample buffer (time-pruned circular buffer). Used by WINDOW and
// OSCILLATION checks. Stores (value, timestamp) pairs and evicts entries older
// than the requested window.
// ---------------------------------------------------------------------------
class DZ_RingBuffer {
public:
    void reset() { _tail = 0; _count = 0; _sum = 0.0f; _has_first = false; }
    void push(float v, uint32_t now_ms);
    void prune(uint32_t now_ms, uint32_t window_ms);
    uint16_t count() const { return _count; }

    // True once the buffer has accumulated at least window_ms of data, so a
    // windowed statistic reflects a complete window rather than a partial one.
    bool full(uint32_t now_ms, uint32_t window_ms) const {
        return _has_first && (now_ms - _first_ms) >= window_ms;
    }

    float mean() const;                // O(1): maintained incrementally
    float peak() const;                // maximum value in the window
    float range() const;               // max - min in the window
    uint16_t mean_crossings() const;   // sign changes of (v - mean)

private:
    float    _v[DZ_CHECK_BUFFER_SAMPLES];
    uint32_t _t[DZ_CHECK_BUFFER_SAMPLES];
    uint16_t _tail = 0;       // index of oldest sample
    uint16_t _count = 0;      // number of valid samples
    float    _sum = 0.0f;     // running sum of live samples (for O(1) mean)
    uint32_t _first_ms = 0;   // timestamp of the first sample since reset
    bool     _has_first = false;
};

// Mutable per-check runtime state (debounce timer + optional sample buffer).
// `buf` is non-null only for WINDOW/OSCILLATION checks; AP_DangerZone::init()
// hands each windowed check a buffer from the vehicle-supplied pool, so no
// buffer memory is reserved for THRESHOLD/DURATION/NONE slots.
struct DZ_CheckState {
    DZ_RingBuffer *buf = nullptr;
    uint32_t raw_since_ms = 0;   // when the raw predicate first became true
    bool     raw_active = false; // is the raw predicate currently latched
    void reset() { if (buf != nullptr) { buf->reset(); } raw_since_ms = 0; raw_active = false; }
};

// ---------------------------------------------------------------------------
// DZ_Check: homogeneous value type with typed factory constructors.
// ---------------------------------------------------------------------------
struct DZ_Check {
    DZ_CheckType type;
    DZ_Getter    getter;
    DZ_Cmp       cmp;
    DZ_Stat      stat;
    float        thresh;
    uint32_t     window_ms;
    uint16_t     min_crossings;
    uint32_t     duration_ms;

    // ---- typed factories (call sites never set `type`) ----
    static constexpr DZ_Check Threshold(DZ_Getter g, DZ_ThresholdParams p) {
        return { DZ_CheckType::THRESHOLD, g, p.cmp, DZ_Stat::MEAN, p.thresh, 0, 0, 0 };
    }
    static constexpr DZ_Check Duration(DZ_Getter g, DZ_DurationParams p) {
        return { DZ_CheckType::DURATION, g, p.cmp, DZ_Stat::MEAN, p.thresh, 0, 0, p.duration_ms };
    }
    static constexpr DZ_Check Window(DZ_Getter g, DZ_WindowParams p) {
        return { DZ_CheckType::WINDOW, g, p.cmp, p.stat, p.thresh, p.window_ms, 0, p.duration_ms };
    }
    static constexpr DZ_Check Oscillation(DZ_Getter g, DZ_OscParams p) {
        return { DZ_CheckType::OSCILLATION, g, DZ_Cmp::ABOVE, DZ_Stat::MEAN, 0, p.window_ms, p.min_crossings, p.duration_ms };
    }

    // Raw (pre-debounce) predicate result for the injected value.
    bool raw_satisfied(DZ_CheckState& st, float value, uint32_t now_ms) const;

    // Full check result, applying the duration_ms debounce. `value` is injected.
    bool update(DZ_CheckState& st, float value, uint32_t now_ms) const;
};

// ---------------------------------------------------------------------------
// DZ_Group: a flat AND/OR group of up to DZ_MAX_CHECKS checks. Unused slots
// are DZ_CheckType::NONE. Built via dz::AND(...) / dz::OR(...).
// ---------------------------------------------------------------------------
struct DZ_Group {
    DZ_Op    op;
    DZ_Check checks[DZ_MAX_CHECKS];

    // Reads each check's getter, updates its state, and combines.
    // `states` is parallel to `checks` (one slot per check). All checks are
    // evaluated every tick (their buffers/timers must advance); only the
    // boolean combination depends on `op`. An empty group is never satisfied.
    // `out_mask` receives a per-check satisfied bitmask
    bool update(DZ_CheckState states[DZ_MAX_CHECKS], uint32_t now_ms,
                uint8_t *out_mask = nullptr) const;
};

namespace dz {
    template <typename... Cs> constexpr DZ_Group OR(Cs... cs) {
        static_assert(sizeof...(Cs) <= DZ_MAX_CHECKS, "too many checks in group");
        return DZ_Group{ DZ_Op::OR, { cs... } };
    }
    template <typename... Cs> constexpr DZ_Group AND(Cs... cs) {
        static_assert(sizeof...(Cs) <= DZ_MAX_CHECKS, "too many checks in group");
        return DZ_Group{ DZ_Op::AND, { cs... } };
    }

    // WINDOW and OSCILLATION are the only checks that need a rolling buffer.
    constexpr bool needs_buffer(DZ_CheckType t) {
        return t == DZ_CheckType::WINDOW || t == DZ_CheckType::OSCILLATION;
    }
    // Count of windowed checks in a group (recursive: C++11 has no constexpr loops).
    constexpr uint16_t count_buffers(const DZ_Group& g, uint8_t i = 0) {
        return i >= DZ_MAX_CHECKS ? 0
            : (needs_buffer(g.checks[i].type) ? 1 : 0) + count_buffers(g, i + 1);
    }
}

#endif  // AP_DANGERZONE_ENABLED
