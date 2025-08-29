-- Adjusts Q_WVANE_GAIN between 3 and 0 based on pitch angle (30° → 3, 60° → 0)
-- Only works in Q (VTOL) modes. Prints gain on change every LOG_INTERVAL seconds.

local pitch_sensor = assert(ahrs.get_pitch, "AHRS:get_pitch() not available")

-- Q-mode numbers
local QMODES = {
    [17] = true,  -- QSTABILIZE
    [18] = true,  -- QHOVER
    [19] = true,  -- QLOITER
    [20] = true,  -- QLAND
    [21] = true,  -- QRTL
    [22] = true,  -- QAUTOTUNE
    [23] = true,  -- QACRO
}

-- Pitch-to-gain mapping boundaries
local PITCH_MIN = math.rad(30)  -- radians
local PITCH_MAX = math.rad(60)
local GAIN_MIN = 0
local GAIN_MAX = 3

-- Timing
local RUN_FREQ = 10
local run_interval = 1.0 / RUN_FREQ

-- Logging state
local last_logged_gain = nil
local last_log_time = 0
local LOG_INTERVAL = 5 -- seconds

-- Clamp helper
local function clamp(val, min, max)
    return math.max(min, math.min(max, val))
end

-- Main function
local function update()

    local mode = vehicle:get_mode()
    if not QMODES[mode] then
        return update, run_interval
    end

    local pitch = (math.rad(90) - math.abs(ahrs:get_pitch()))

    local gain
    if pitch <= PITCH_MIN then
        gain = GAIN_MAX
    elseif pitch >= PITCH_MAX then
        gain = GAIN_MIN
    else
        local t = (pitch - PITCH_MIN) / (PITCH_MAX - PITCH_MIN)
        gain = GAIN_MAX * (1 - t)
    end

    gain = clamp(gain, GAIN_MIN, GAIN_MAX)

    -- Set Q_WVANE_GAIN directly
    param:set("Q_WVANE_GAIN", gain)

    -- Log gain value every 5 seconds if changed
    local now = millis() / 1000
    if (gain ~= last_logged_gain) and (now - last_log_time >= LOG_INTERVAL) then
        gcs:send_text(6, string.format("Q_WVANE_GAIN changed to %.2f", gain))
        last_logged_gain = gain
    end

    return update, run_interval
end

gcs:send_text(6, "Airbound Lua: auto Q_WVANE_GAIN by pitch loaded.")

return update, run_interval
