--[[
   Axis 2 (independent vectoring) trim switcher for dual-axis tiltrotor quadplanes (Q_TILT_TYPE=4).  V1.4

   The left/right Axis 2 vectoring servos (SERVO_FUNCTION 190/191) need a
   different neutral (SERVOx_TRIM) in VTOL flight vs fixed wing flight.
   This script applies VECTRIM_FW_L/VECTRIM_FW_R while fixed wing and
   VECTRIM_VT_L/VECTRIM_VT_R otherwise.

   The VTOL trims are stored in VECTRIM_VT_L/R, not in SERVOx_TRIM. SERVOx_TRIM
   on these two channels is owned by this script: it is forced to the trim for
   the current flight state on (re)start, so a FW trim that ended up saved in
   SERVOx_TRIM (GCS edit or param file captured while FW trims were active)
   cannot become the VTOL trim.
   Tune with VECTRIM_VT_* / VECTRIM_FW_*, not SERVOx_TRIM.

   The fixed wing trims are only applied once Axis 1 is fully forward,
   read from quadplane:get_tilt() (Tiltrotor current_tilt, 0 = vertical,
   1 = fully forward), so the VTOL trim is kept for the whole forward
   transition while the rotors tilt. Requires firmware with the
   quadplane:get_tilt() scripting binding.

   On a script runtime error the scripting engine stops this script and the
   trims stay as they were until the scripts are restarted or the board reboots.
--]]

local MAV_SEVERITY_ERROR   = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO    = 6

local PARAM_TABLE_KEY = 101   -- change if this collides with another loaded script's table key
local PARAM_TABLE_PREFIX = "VECTRIM_"

-- bind a parameter to a variable
local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('VecTrim: could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('VecTrim: could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'VecTrim: could not add param table')

--[[
  // @Param: VECTRIM_ENABLE
  // @DisplayName: Axis2 trim switcher enable
  // @Description: Enables switching Axis 2 (independent vectoring) servo trims between VTOL and fixed wing flight
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local VECTRIM_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: VECTRIM_FW_L
  // @DisplayName: Axis2 left vectoring fixed wing trim
  // @Description: SERVOx_TRIM PWM applied to the left Axis 2 vectoring servo (SERVO_FUNCTION 190) while in fixed wing flight
  // @Units: PWM
  // @Range: 800 2200
  // @Increment: 1
  // @User: Standard
--]]
local VECTRIM_FW_L = bind_add_param('FW_L', 2, 1500)

--[[
  // @Param: VECTRIM_FW_R
  // @DisplayName: Axis2 right vectoring fixed wing trim
  // @Description: SERVOx_TRIM PWM applied to the right Axis 2 vectoring servo (SERVO_FUNCTION 191) while in fixed wing flight
  // @Units: PWM
  // @Range: 800 2200
  // @Increment: 1
  // @User: Standard
--]]
local VECTRIM_FW_R = bind_add_param('FW_R', 3, 1500)

--[[
  // @Param: VECTRIM_VT_L
  // @DisplayName: Axis2 left vectoring VTOL trim
  // @Description: SERVOx_TRIM PWM applied to the left Axis 2 vectoring servo (SERVO_FUNCTION 190) in VTOL flight. This, not SERVOx_TRIM, is the stored VTOL trim. 0 = capture from SERVOx_TRIM on next boot
  // @Units: PWM
  // @Range: 0 2200
  // @Increment: 1
  // @User: Standard
--]]
local VECTRIM_VT_L = bind_add_param('VT_L', 4, 0)

--[[
  // @Param: VECTRIM_VT_R
  // @DisplayName: Axis2 right vectoring VTOL trim
  // @Description: SERVOx_TRIM PWM applied to the right Axis 2 vectoring servo (SERVO_FUNCTION 191) in VTOL flight. This, not SERVOx_TRIM, is the stored VTOL trim. 0 = capture from SERVOx_TRIM on next boot
  // @Units: PWM
  // @Range: 0 2200
  // @Increment: 1
  // @User: Standard
--]]
local VECTRIM_VT_R = bind_add_param('VT_R', 5, 0)

local K_TILTMOTOR_LEFT_VEC  = 190
local K_TILTMOTOR_RIGHT_VEC = 191

local TILT_FULL_FWD = 0.999   -- current_tilt >= this is treated as Axis 1 fully forward

local UPDATE_PERIOD_MS = 200

-- check the firmware has the quadplane:get_tilt() binding before doing anything
if not pcall(function() return quadplane:get_tilt() end) then
   gcs:send_text(MAV_SEVERITY_ERROR, "VecTrim: quadplane:get_tilt() binding missing, stopping")
   return
end

-- resolve the SERVOx_TRIM parameter name for a given SERVO_FUNCTION, or nil if not assigned
local function trim_param_name(servo_function)
   local chan = SRV_Channels:find_channel(servo_function)
   if not chan then
      return nil
   end
   return string.format("SERVO%d_TRIM", chan + 1)
end

local left_trim_param  = trim_param_name(K_TILTMOTOR_LEFT_VEC)
local right_trim_param = trim_param_name(K_TILTMOTOR_RIGHT_VEC)

if not left_trim_param or not right_trim_param then
   gcs:send_text(MAV_SEVERITY_ERROR, "VecTrim: Axis2 left/right servo function not assigned, stopping")
   return
end

-- SERVOx_TRIM is an integer param, keep everything in whole PWM so comparisons are exact
local function pwm(v)
   return math.floor(v + 0.5)
end

-- first run: capture the VTOL trims from SERVOx_TRIM once (at boot, before any FW trim has been applied)
if VECTRIM_VT_L:get() <= 0 then
   VECTRIM_VT_L:set_and_save(pwm(param:get(left_trim_param)))
   gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: VT_L captured from %s = %.0f", left_trim_param, VECTRIM_VT_L:get()))
end
if VECTRIM_VT_R:get() <= 0 then
   VECTRIM_VT_R:set_and_save(pwm(param:get(right_trim_param)))
   gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: VT_R captured from %s = %.0f", right_trim_param, VECTRIM_VT_R:get()))
end

-- the trim values this script last wrote, used to detect external writes to SERVOx_TRIM
local expect_left = nil
local expect_right = nil

-- true once the fixed wing trims have been applied
local in_fw_trim = false

local function set_trims(l, r)
   expect_left, expect_right = pwm(l), pwm(r)
   param:set(left_trim_param, expect_left)
   param:set(right_trim_param, expect_right)
end

local function restore_vtol_trims(reason)
   set_trims(VECTRIM_VT_L:get(), VECTRIM_VT_R:get())
   in_fw_trim = false
   gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: %s, VTOL trims applied (L=%d R=%d)",
                  reason, expect_left, expect_right))
end

local function apply_fw_trims(tilt)
   set_trims(VECTRIM_FW_L:get(), VECTRIM_FW_R:get())
   in_fw_trim = true
   gcs:send_text(MAV_SEVERITY_INFO, string.format("VecTrim: Axis1 %.0fdeg, FW trims applied (L=%d R=%d)",
                  tilt * 90, expect_left, expect_right))
end

-- on (re)start, force the trims for the current state regardless of what is stored in SERVOx_TRIM,
-- so a script/engine restart in fixed wing flight does not drop to VTOL trims
if VECTRIM_ENABLE:get() > 0 and (not quadplane:in_vtol_mode()) and quadplane:get_tilt() >= TILT_FULL_FWD then
   apply_fw_trims(quadplane:get_tilt())
else
   restore_vtol_trims("startup")
end

local function update()
   if VECTRIM_ENABLE:get() <= 0 then
      if in_fw_trim then
         restore_vtol_trims("disabled")
      end
      return update, UPDATE_PERIOD_MS
   end

   -- SERVOx_TRIM on these channels is owned by this script; warn and re-assert if something else changed it
   if pwm(param:get(left_trim_param)) ~= expect_left or pwm(param:get(right_trim_param)) ~= expect_right then
      gcs:send_text(MAV_SEVERITY_WARNING, "VecTrim: SERVOx_TRIM changed externally, tune VECTRIM_VT_*/FW_* instead")
      set_trims(expect_left, expect_right)
   end

   local is_vtol   = quadplane:in_vtol_mode()
   local tilt      = quadplane:get_tilt()
   local axis1_fwd = tilt >= TILT_FULL_FWD

   -- FW trims only in fixed wing flight with Axis 1 fully forward, VTOL trims otherwise
   local want_fw = (not is_vtol) and axis1_fwd

   if want_fw and not in_fw_trim then
      apply_fw_trims(tilt)
   elseif not want_fw and in_fw_trim then
      restore_vtol_trims(is_vtol and "VTOL mode" or "Axis1 not fwd")
   end

   return update, UPDATE_PERIOD_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "AB VecTrim V1.3: loaded")

return update, UPDATE_PERIOD_MS
