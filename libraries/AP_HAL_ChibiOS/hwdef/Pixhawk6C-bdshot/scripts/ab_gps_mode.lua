-- GPS management in Q modes: sets GPS[1] as primary while it has a 3D fix
-- switches to GPS[0] if sats or vacc drop below a threshold
-- switches back to GPS[1] if sats and vacc rise above a threshold

local run_interval = 1.0 -- once every run_interval seconds

local VTOL_GPS = 1
local FW_GPS  = 0

-- Asymmetric thresholds give hysteresis.
local SAT_MAX_THRESHOLD  = 10  -- switch AWAY when sats < this
local SAT_MIN_THRESHOLD  = 15  -- switch BACK only when sats >= this
local VACC_MIN_THRESHOLD = 2.0 -- switch AWAY when vacc > this
local VACC_MAX_THRESHOLD = 1.5 -- switch BACK only when vacc <= this

local DEFAULT_PRIMARY = 1 -- 0 is FW GPS, 1 is VTOL GPS
local DEFAULT_GPS_AUTO_SWITCH = 1 -- USE_BEST
local VTOL_GPS_AUTO_SWITCH = 4 -- USE_PRIMARY_IF_3D_FIX


local function update()
    local in_vtol = quadplane ~= nil and quadplane:in_vtol_mode()
    local current_gps_auto_switch = param:get("GPS_AUTO_SWITCH")

    if not in_vtol then
        if DEFAULT_GPS_AUTO_SWITCH ~= nil and current_gps_auto_switch ~= DEFAULT_GPS_AUTO_SWITCH then
            gcs:send_text(6, "Not in VTOL phase, resetting GPS settings to default")
            param:set("GPS_AUTO_SWITCH", DEFAULT_GPS_AUTO_SWITCH)
            param:set("GPS_PRIMARY", DEFAULT_PRIMARY)
            gcs:send_text(6, string.format("GPS_AUTO_SWITCH: %d", DEFAULT_GPS_AUTO_SWITCH))
            gcs:send_text(6, string.format("GPS[%d] set as primary", DEFAULT_PRIMARY))
        end
        return update, run_interval * 1000
    end

    -- Q mode
    if current_gps_auto_switch ~= nil and current_gps_auto_switch ~= VTOL_GPS_AUTO_SWITCH then
        gcs:send_text(6, "Q mode, GPS autoswitch set to use primary")
        param:set("GPS_AUTO_SWITCH", VTOL_GPS_AUTO_SWITCH)
    end

    local current_primary = param:get("GPS_PRIMARY")
    if current_primary == nil then return update, run_interval * 1000 end

    -- Make sure both GPS instances exist
    if gps:num_sensors() <= VTOL_GPS then return update, run_interval * 1000 end

    local vtol_sats = gps:num_sats(VTOL_GPS)
    local vtol_vacc = gps:vertical_accuracy(VTOL_GPS) or 0
    
    if current_primary == VTOL_GPS then
        -- on preferred GPS
        if vtol_sats < SAT_MAX_THRESHOLD or vtol_vacc > VACC_MIN_THRESHOLD then
            gcs:send_text(6, string.format(
                "Q mode, switching to GPS[%d]: GPS[%d] sats=%d vacc=%.2f",
                FW_GPS, VTOL_GPS, vtol_sats, vtol_vacc))
            param:set("GPS_PRIMARY", FW_GPS)
        end
    else
        -- on fallback GPS
        if vtol_sats >= SAT_MIN_THRESHOLD and vtol_vacc <= VACC_MAX_THRESHOLD then
            gcs:send_text(6, string.format(
                "Q mode, returning to GPS[%d]: sats=%d vacc=%.2f",
                VTOL_GPS, vtol_sats, vtol_vacc))
            param:set("GPS_PRIMARY", VTOL_GPS)
        end
    end

    return update, run_interval * 1000
end

gcs:send_text(6, "Airbound Lua: GPS switch in Q modes script loaded")
gcs:send_text(6, string.format("GPS default primary: %d", DEFAULT_PRIMARY))
gcs:send_text(6, string.format("GPS default auto switch setting: %d", DEFAULT_GPS_AUTO_SWITCH))

return update, run_interval * 1000