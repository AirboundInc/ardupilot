--[[
  RTL airspeed override for quadplanes.

  While the vehicle is in RTL, this script watches distance-to-home and:
   - inside RTLAS_DIST1 metres, overrides AIRSPEED_CRUISE/AIRSPEED_MIN with
     RTLAS_CRUISE/RTLAS_MIN (eg. to slow down for the VTOL transition)
   - inside RTLAS_DIST2 metres, switches the vehicle to QSTABILIZE and holds
     it there for at least RTLAS_HOLD seconds, then waits for the pitch
     angle to settle to within RTLAS_PIT_ERR degrees of level before
     switching on to QLOITER, forcing the QLOITER switch anyway after
     RTLAS_TIMEOUT seconds (from the QSTABILIZE switch) if pitch never
     settles

  The original AIRSPEED_CRUISE/AIRSPEED_MIN values are captured on entry to
  RTL and restored as soon as the vehicle leaves the RTL/QSTABILIZE hold
  managed by this script (including the eventual QLOITER switch), so
  nothing is permanently changed.
--]]

---@diagnostic disable: param-type-mismatch

assert(quadplane, "RTL airspeed override requires a quadplane")

local PARAM_TABLE_KEY = 102
local PARAM_TABLE_PREFIX = "RTLAS_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), "could not add param table")

function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: RTLAS_ENABLE
  // @DisplayName: RTL airspeed override enable
  // @Description: Enable RTL distance-based airspeed override and QSTABILIZE/QLOITER switch
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local RTLAS_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: RTLAS_CRUISE
  // @DisplayName: RTL override cruise airspeed
  // @Description: AIRSPEED_CRUISE value applied once within RTLAS_DIST1 of home during RTL
  // @Units: m/s
  // @Range: 5 100
  // @User: Standard
--]]
local RTLAS_CRUISE = bind_add_param("CRUISE", 2, 12)

--[[
  // @Param: RTLAS_MIN
  // @DisplayName: RTL override minimum airspeed
  // @Description: AIRSPEED_MIN value applied once within RTLAS_DIST1 of home during RTL
  // @Units: m/s
  // @Range: 5 100
  // @User: Standard
--]]
local RTLAS_MIN = bind_add_param("MIN", 3, 12)

--[[
  // @Param: RTLAS_DIST1
  // @DisplayName: RTL airspeed override distance
  // @Description: Distance to home during RTL at which AIRSPEED_CRUISE/AIRSPEED_MIN are overridden
  // @Units: m
  // @Range: 0 1000
  // @User: Standard
--]]
local RTLAS_DIST1 = bind_add_param("DIST1", 4, 400)

--[[
  // @Param: RTLAS_DIST2
  // @DisplayName: RTL QSTABILIZE switch distance
  // @Description: Distance to home during RTL at which the vehicle is switched to QSTABILIZE, ahead of the QLOITER switch
  // @Units: m
  // @Range: 0 1000
  // @User: Standard
--]]
local RTLAS_DIST2 = bind_add_param("DIST2", 5, 100)

--[[
  // @Param: RTLAS_HOLD
  // @DisplayName: RTL QSTABILIZE minimum hold time
  // @Description: Minimum time to hold QSTABILIZE after RTLAS_DIST2 before the pitch angle is even checked for the QLOITER switch
  // @Units: s
  // @Range: 0 30
  // @User: Standard
--]]
local RTLAS_HOLD = bind_add_param("HOLD", 6, 3)

--[[
  // @Param: RTLAS_PIT_ERR
  // @DisplayName: RTL QSTABILIZE pitch settle threshold
  // @Description: Once RTLAS_HOLD has elapsed, the vehicle is switched on to QLOITER once the pitch angle comes within this many degrees of level
  // @Units: deg
  // @Range: 0 45
  // @User: Standard
--]]
local RTLAS_PIT_ERR = bind_add_param("PIT_ERR", 7, 10)

--[[
  // @Param: RTLAS_TIMEOUT
  // @DisplayName: RTL QSTABILIZE pitch settle timeout
  // @Description: Maximum time (from the QSTABILIZE switch, inclusive of RTLAS_HOLD) to wait for pitch to settle within RTLAS_PIT_ERR before switching on to QLOITER regardless
  // @Units: s
  // @Range: 0 60
  // @User: Standard
--]]
local RTLAS_TIMEOUT = bind_add_param("TIMEOUT", 8, 10)

local airspeed_cruise = Parameter("AIRSPEED_CRUISE")
local airspeed_min = Parameter("AIRSPEED_MIN")

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local MODE_RTL = 11
local MODE_QSTABILIZE = 17
local MODE_QLOITER = 19

gcs:send_text(MAV_SEVERITY.INFO, "Jugaad RTLAS: RTL airspeed override script loaded")

-- state for the RTL episode currently in progress, nil when not in RTL
local original_cruise = nil
local original_min = nil
local override_applied = false
-- true once this script has switched the vehicle to QSTABILIZE and is
-- waiting for pitch to settle before handing off to QLOITER
local qstabilize_triggered = false
-- millis() timestamp of the QSTABILIZE switch, used for RTLAS_TIMEOUT
local qstabilize_start_ms = nil

local function restore_airspeed()
   if override_applied then
      airspeed_cruise:set(original_cruise)
      airspeed_min:set(original_min)
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RTLAS: restored airspeed cruise %.1f min %.1f", original_cruise, original_min))
   end
   original_cruise = nil
   original_min = nil
   override_applied = false
   qstabilize_triggered = false
   qstabilize_start_ms = nil
end

function update()
   local mode = vehicle:get_mode()

   if RTLAS_ENABLE:get() <= 0 then
      -- disabled, make sure nothing is left overridden
      restore_airspeed()
      return update, 200
   end

   if qstabilize_triggered then
      -- we've already switched to QSTABILIZE ourselves and are waiting
      -- for pitch to settle before handing off to QLOITER
      if mode ~= MODE_QSTABILIZE then
         -- something else changed the mode away from our hold (pilot,
         -- failsafe, etc.), abandon our state and stop overriding
         restore_airspeed()
         return update, 200
      end

      local elapsed_s = (millis() - qstabilize_start_ms):tofloat() * 0.001

      if elapsed_s < RTLAS_HOLD:get() then
         -- still within the minimum QSTABILIZE hold, don't even look at
         -- pitch yet
         return update, 200
      end

      local pitch_error_deg = math.abs(math.deg(ahrs:get_pitch()))
      if pitch_error_deg < RTLAS_PIT_ERR:get() then
         vehicle:set_mode(MODE_QLOITER)
         gcs:send_text(MAV_SEVERITY.INFO, string.format("RTLAS: pitch settled (%.1f deg), switching to QLOITER", pitch_error_deg))
      elseif elapsed_s > RTLAS_TIMEOUT:get() then
         vehicle:set_mode(MODE_QLOITER)
         gcs:send_text(MAV_SEVERITY.WARNING, string.format("RTLAS: pitch settle timed out (%.1f deg), switching to QLOITER", pitch_error_deg))
      end

      return update, 200
   end

   if mode ~= MODE_RTL then
      -- not in RTL, restore anything we changed and wait
      restore_airspeed()
      return update, 200
   end

   if original_cruise == nil then
      -- just entered RTL (or script started mid-RTL), capture current values
      original_cruise = airspeed_cruise:get()
      original_min = airspeed_min:get()
   end

   local pos = ahrs:get_relative_position_NED_home()
   if not pos then
      return update, 200
   end
   local dist = pos:xy():length()

   if not override_applied and dist < RTLAS_DIST1:get() then
      airspeed_cruise:set(RTLAS_CRUISE:get())
      airspeed_min:set(RTLAS_MIN:get())
      override_applied = true
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RTLAS: airspeed overridden, cruise %.1f min %.1f", RTLAS_CRUISE:get(), RTLAS_MIN:get()))
   end

   if dist < RTLAS_DIST2:get() then
      qstabilize_triggered = true
      qstabilize_start_ms = millis()
      vehicle:set_mode(MODE_QSTABILIZE)
      gcs:send_text(MAV_SEVERITY.INFO, "RTLAS: switching to QSTABILIZE, waiting for pitch to settle")
   end

   return update, 200
end

return update()
