-- Tailsitter Recovery Loop Script (Fixed for Firmware Compatibility)
-- Logic: Bad Pitch -> Save Mode -> GUIDED (Auto-Hover) -> Wait -> Return to Saved Mode

local LOOP_MS = 100 -- Run at 10Hz

-- 1. SETUP PARAMETER TABLE
local KEY = 75
assert(param:add_table(KEY, "QREC_", 20), "QREC table failed")

-- 2. ADD PARAMETERS
param:add_param(KEY, 2,  "MIN_BAD_PIT", 40)    -- Pitch limit
param:add_param(KEY, 10, "ENABLE",      1)    -- 1 = Enabled
param:add_param(KEY, 11, "MODE_DLY",    1000) -- Delay (ms) before checking pitch
param:add_param(KEY, 12, "BAD_CNT",     3)    -- Bad ticks required

-- 3. BIND PARAMETERS
local function bind_param(name)
    local p = Parameter(name)
    if not p then
        gcs:send_text(2, "QREC: Reboot Required for " .. name)
    end
    return p
end

local p_enable      = bind_param("QREC_ENABLE")
local p_min_bad_pit = bind_param("QREC_MIN_BAD_PIT")
local p_mode_dly    = bind_param("QREC_MODE_DLY")
local p_bad_req     = bind_param("QREC_BAD_CNT")

-- 4. MODE DEFINITIONS (ArduPlane)
local MODE_QLOITER = 19 -- Auto-Hover (Functionally same as QLoiter 50% Thr)
local MODE_QLAND  = 20

-- 5. STATE VARIABLES
local active = false
local bad_count = 0
local last_mode_idx = 0
local mode_entry_time = 0

-- Helper: Radians to Degrees
local function rad2deg(r) return r * 57.2958 end

function is_vehicle_landing()
    local current_mode = vehicle:get_mode()
    if current_mode == MODE_QLAND then
        return true
    elseif quadplane:in_vtol_land_descent() then
        return true
    else
        return false
    end
end


function update()
    -- Safety Check
    if not p_enable  then return update, 1000 end
    if p_enable:get() ~= 1 then return update, LOOP_MS end

    local current_mode = vehicle:get_mode()
    if not current_mode then return update, LOOP_MS end
    
    local now = millis()

    -- Detect Mode Changes
    if current_mode ~= last_mode_idx then
        last_mode_idx = current_mode
        mode_entry_time = now
        bad_count = 0 
    end

    -- ==========================================================
    -- LOGIC 1: MONITORING (Checking Pitch)
    -- ==========================================================
    if not active then
        if is_vehicle_landing() then
            
            -- Wait for Delay (settle time)
            local delay_ms = p_mode_dly:get() or 1000
            if (now - mode_entry_time) > delay_ms then

                local pitch_deg = rad2deg(ahrs:get_pitch() or 0)
                local threshold = p_min_bad_pit:get() or 0
                if pitch_deg < threshold then
                    bad_count = bad_count + 1
                    local required_count = p_bad_req:get() or 1
                    
                    if bad_count % 10 == 0 then
                        gcs:send_text(6, string.format("QREC: Bad Pitch %.1f < %.1f", pitch_deg, threshold))
                    end

                    if bad_count >= required_count then
                        if vehicle:set_mode(MODE_QLOITER) then
                            active = true
                            bad_count = 0
                            gcs:send_text(2, "QREC: Switching to QLoiter" )
                        end
                    end
                else
                    bad_count = 0 
                end
            end
        end

    -- ==========================================================
    -- LOGIC 2: RECOVERING (Waiting in GUIDED)
    -- ==========================================================
    elseif active then
        
        -- Cancel if pilot manually switched mode
        if current_mode ~= MODE_QLOITER then
            active = false
            gcs:send_text(6, "QREC: Manual Override Detected")
        end
    end
    return update, LOOP_MS

end

gcs:send_text(6, "QREC: Loaded Autobail script")
return update, LOOP_MS