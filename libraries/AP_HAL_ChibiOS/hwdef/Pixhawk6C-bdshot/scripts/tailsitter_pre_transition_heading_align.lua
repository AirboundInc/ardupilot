-- Tailsitter Pre-Transition Heading Alignment (v9.0 - NO MODE CHANGES)
-- Aligns yaw to next waypoint bearing while remaining in AUTO
-- Uses yaw-rate control only

-- ================= CONFIG =================
local UPDATE_RATE_MS = 200
local HEADING_TOLERANCE_DEG = 6
local MAX_YAW_RATE_DEG_S = 25
local ALIGN_TIMEOUT_MS = 12000
local MIN_ALTITUDE_M = 10
local AUTO_STABLE_TIME_MS = 5000
local DEBUG_INTERVAL_MS = 3000

-- ================= STATES =================
local STATE_IDLE = 0
local STATE_MONITORING = 1
local STATE_ALIGNING = 2

local state = STATE_IDLE
local target_heading = 0
local target_wp_idx = -1
local last_aligned_wp = -1
local align_start_time = 0
local last_auto_time = 0
local last_debug_time = 0

-- ================= MODES =================
local MODE_AUTO = 10

-- ================= HELPERS =================
local function wrap_360(angle)
    local res = angle % 360
    if res < 0 then res = res + 360 end
    return res
end

local function heading_error_deg(current, target)
    local err = target - current
    if err > 180 then err = err - 360 end
    if err < -180 then err = err + 360 end
    return err
end

local function clamp(x, minv, maxv)
    if x < minv then return minv end
    if x > maxv then return maxv end
    return x
end

local function is_invalid_loc(lat, lng)
    return (math.abs(lat) < 100 and math.abs(lng) < 100)
end

local function is_airborne()
    local v = ahrs:get_velocity_NED()
    if not v then return false end
    local speed = math.sqrt(v:x()^2 + v:y()^2 + v:z()^2)
    return speed > 0.5
end

-- Get next NAV waypoint bearing
local function get_next_wp_heading()
    local curr_loc = ahrs:get_position()
    if not curr_loc then return nil, nil, "No GPS" end

    local idx = mission:get_current_nav_index()
    if not idx or idx == 0 or idx == 65535 then return nil, nil, "No mission" end

    for i = idx, idx + 6 do
        local item = mission:get_item(i)
        if item then
            local cmd = item:command()
            local lat = item:x()
            local lng = item:y()
            if cmd == 16 and not is_invalid_loc(lat, lng) then -- NAV waypoint
                local target_loc = Location()
                target_loc:lat(lat)
                target_loc:lng(lng)
                local bearing = math.deg(curr_loc:get_bearing(target_loc))
                return wrap_360(bearing), i, "OK"
            end
        end
    end

    return nil, nil, "No valid WP"
end

local function next_leg_is_fixed_wing()
    local idx = mission:get_current_nav_index()
    if not idx then return false end

    for i = idx, idx + 3 do
        local item = mission:get_item(i)
        if item then
            local cmd = item:command()
            if cmd == 16 then return true end
        end
    end
    return false
end

-- ================= MAIN LOOP =================
function update()
    -- Only operate in VTOL
    if not quadplane:in_vtol_mode() then
        state = STATE_IDLE
        return update, UPDATE_RATE_MS
    end

    local now = millis()
    local mode = vehicle:get_mode()

    -- Track AUTO entry time
    if mode == MODE_AUTO and last_auto_time == 0 then
        last_auto_time = now
    elseif mode ~= MODE_AUTO then
        last_auto_time = 0
        state = STATE_IDLE
    end

    if state == STATE_IDLE then
        if mode == MODE_AUTO and arming:is_armed() then
            state = STATE_MONITORING
            gcs:send_text(6, "Aligner: Monitoring (AUTO)")
        end

    elseif state == STATE_MONITORING then
        if mode ~= MODE_AUTO then
            state = STATE_IDLE
            return update, UPDATE_RATE_MS
        end

        -- SAFETY GATES
        local pos = ahrs:get_position()
        if not pos then return update, UPDATE_RATE_MS end
        local alt_m = pos:alt() * 0.01

        if (now - last_auto_time) < AUTO_STABLE_TIME_MS then
            return update, UPDATE_RATE_MS -- too soon after entering AUTO
        end

        if alt_m < MIN_ALTITUDE_M then
            return update, UPDATE_RATE_MS -- not high enough
        end

        if not is_airborne() then
            return update, UPDATE_RATE_MS -- not airborne
        end

        if next_leg_is_fixed_wing() then
            local heading, idx, err = get_next_wp_heading()
            if heading and idx ~= last_aligned_wp then
                target_heading = heading
                target_wp_idx = idx
                align_start_time = now
                state = STATE_ALIGNING
                gcs:send_text(6, string.format("Aligner: Aligning to %.0f° (WP %d)", heading, idx))
            end
        end

    elseif state == STATE_ALIGNING then
        -- Timeout protection
        if (now - align_start_time) > ALIGN_TIMEOUT_MS then
            gcs:send_text(4, "Aligner: Timeout → Stop aligning")
            last_aligned_wp = target_wp_idx
            state = STATE_IDLE
            return update, UPDATE_RATE_MS
        end

        local curr_yaw = wrap_360(math.deg(ahrs:get_yaw()))
        local err = heading_error_deg(curr_yaw, target_heading)
        local yaw_rate = clamp(err * 1.5, -MAX_YAW_RATE_DEG_S, MAX_YAW_RATE_DEG_S)

        -- Yaw-rate command only (no mode change, no position override)
        vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, true, yaw_rate)

        if math.abs(err) < HEADING_TOLERANCE_DEG then
            gcs:send_text(6, "Aligner: Heading aligned")
            last_aligned_wp = target_wp_idx
            state = STATE_IDLE
        end
    end

    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "Aligner v9.0 Loaded (NO MODE CHANGES)")
return update()