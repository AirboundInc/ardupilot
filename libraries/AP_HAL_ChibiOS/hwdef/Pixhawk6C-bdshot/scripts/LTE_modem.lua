--[[
    driver for LTE modems with AT command set
    supported chipsets:
    - SIM7600, SIMCom R1951
    - EC200, EC20, EC25
    - Air780, BG95, EG800Q

    Unified Fast Reconnect / Advanced Diagnostics Flow:
    1. Both Quectel and SimCom modules trigger a specialized 'fast_connect' flow.
    2. Generic modules safely bypass specific AT commands.

    Fixes applied:
    Fix 1: Smart Watchdog. Only GCS data feeds it during CONNECTED.
    Fix 2: Parameterized Timers. LTE_GRACE handles CEREG drops, LTE_STUCK_T handles setup timeouts.
    Fix 3: CIPOPEN 3x failure goes to CREG (soft reconnect). 3 soft reconnects trigger hard reset.
    Fix 4: cops_rescanning guard removed. Gentle CREG polling logic applied.
    Fix 5: Silenced Log Spam & replaced Loop Counters with absolute millis() tracking.
    Fix 6: CSV Parsing respects empty commas (.-) to prevent telemetry column shifting.
    Fix 7: Outage time backdated by TIMEOUT, skip SIGNAL_GATE after reset, 1s CPIN wait.
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "LTE_"
local PPP = 48

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'LTE_modem: could not add param table')

local P = {
    ENABLE      = bind_add_param('ENABLE',  1, 1),
    SERPORT     = bind_add_param('SERPORT',  2, 0),
    SCRPORT     = bind_add_param('SCRPORT',  3, 0),
    SERVER_IP0  = bind_add_param('SERVER_IP0',  4, 0),
    SERVER_IP1  = bind_add_param('SERVER_IP1',  5, 0),
    SERVER_IP2  = bind_add_param('SERVER_IP2',  6, 0),
    SERVER_IP3  = bind_add_param('SERVER_IP3',  7, 0),
    SERVER_PORT = bind_add_param('SERVER_PORT',  8, 0),
    BAUD        = bind_add_param('BAUD',  9, 115200),
    TIMEOUT     = bind_add_param('TIMEOUT', 10, 30),
    PROTOCOL    = bind_add_param('PROTOCOL', 11, 2),
    OPTIONS     = bind_add_param('OPTIONS', 12, 3),
    IBAUD       = bind_add_param('IBAUD', 13, 115200),
    MCCMNC      = bind_add_param('MCCMNC', 14, -1),
    TX_RATE     = bind_add_param('TX_RATE',  20, 0),
    BAND        = bind_add_param('BAND', 21, -1),
    GRACE       = bind_add_param('GRACE', 22, 3), 
    STUCK_T     = bind_add_param('STUCK_T', 23, 15),
    TX_DEAD     = bind_add_param('TX_DEAD', 24, 5)   -- consecutive QISEND stalls before declaring socket dead (0=disable)
}

local supports_routing = networking and networking.add_route -- luacheck: ignore 143
local P_ROUTE = {}
if supports_routing then
    P_ROUTE.IP0  = bind_add_param('ROUTE_IP0',  15, 0)
    P_ROUTE.IP1  = bind_add_param('ROUTE_IP1',  16, 0)
    P_ROUTE.IP2  = bind_add_param('ROUTE_IP2',  17, 0)
    P_ROUTE.IP3  = bind_add_param('ROUTE_IP3',  18, 0)
    P_ROUTE.MASK = bind_add_param('ROUTE_MASK',  19, 32)
end

local OPT = { LOGALL=(1<<0), SIGNALS=(1<<1), NOMUX=(1<<2), NOSIGQUERY=(1<<3), TCP=(1<<4), DPUSH=(1<<5) }

local modem_list = {
    ["SimCom"] = { banner = 'SIMCOM', cmux = 'AT+CMUX=0\r\n', setbaud = 'AT+IPR=%u\r\n', pppopen = 'ATD*99#\r', cpin = 'AT+CPIN?\r\n', cpsi = 'AT+CPSI?\r\n', reset = 'AT+CFUN=1,1\r\n', cipmode = 'AT+CIPMODE=1\r\n', cipopen_udp = 'AT+CIPOPEN=0,"UDP","%d.%d.%d.%d",%d,6001\r\n', cipopen_tcp = 'AT+CIPOPEN=0,"TCP","%d.%d.%d.%d",%d\r\n', cipclose = 'AT+CIPCLOSE=0\r\n', cgerep = 'AT+CGEREP=1,1\r\n', netopen = 'AT+NETOPEN\r\n', mccmnc = 'AT+COPS=1,2,"%u"\r\n', setband_mask = 'AT+CNBP=,0x%x\r\n', setband_all = 'AT+CNBP=,0x480000000000000000000000000000000000000000000042000007FFFFDF3FFF\r\n', config_extra = 'ATH\r\n', fast_connect = true, sim_probe = 'AT+CICCID\r\n', csq_gate = 'AT+CPSI?\r\n', socket_state = 'AT+CIPOPEN?\r\n' },
    ["SimCom2"] = { banner = 'R1951', cmux = 'AT+CMUX=0\r\n', setbaud = 'AT+IPR=%u\r\n', pppopen = 'ATD*99#\r', cpin = 'AT+CPIN?\r\n', cpsi = 'AT+CPSI?\r\n', cipmode = 'AT+CACID=0\r\n', cipopen_tcp = 'AT+CAOPEN=0,0,"TCP","%d.%d.%d.%d",%d\r\n', cipopen_udp = 'AT+CAOPEN=0,0,"UDP","%d.%d.%d.%d",%d\r\n', cgact = 'AT+CGACT?\r\n', cgerep = 'AT+CGEREP=1,1\r\n', reset = 'AT+CFUN=1,1\r\n', netopen = "AT+CNACT=0,1\r\n", netclose = "AT+CNACT=0,0\r\n", cfun = 'AT+CFUN=1\r\n', reset_not_baudrate = true, mccmnc = 'AT+COPS=4,2,"%u"\r\n', caswitch = 'AT+CASWITCH=0,1\r\n', setband = 'AT+CBANDCFG="CAT-M",%d\r\n', setband_all = 'AT+CBANDCFG="CAT-M",1,2,3,4,5,8,12,13,14,18,19,20,25,26,27,28,66,85\r\n' },
    ["Air780"] = { banner = 'AirM2M_780E', cmux = nil, setbaud = 'AT+IPR=%u\r\n', cgact = 'AT+CGACT=1,1\r\n', pppopen = 'ATD*99#\r', cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipmode = 'AT+CIPMODE=1\r\n' },
    ["EC200"] = { banner = 'EC200', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n', fast_connect = true, sim_probe = 'AT+QCCID\r\n', csq_gate = 'AT+QCSQ\r\n', socket_state = 'AT+QISTATE?\r\n' },
    ["BG95"] = { banner = 'BG95', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n', fast_connect = true, sim_probe = 'AT+QCCID\r\n', csq_gate = 'AT+QCSQ\r\n', socket_state = 'AT+QISTATE?\r\n' },
    ["EG800Q"] = { banner = 'EG800Q', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n', fast_connect = true, sim_probe = 'AT+QCCID\r\n', csq_gate = 'AT+QCSQ\r\n', socket_state = 'AT+QISTATE?\r\n' },
    ["EC20"] = { banner = 'EC20C', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,%x,0\r\n', setband_all  = 'AT+QCFG="band",0,7FFFFFFFFFFFFFFF,0\r\n', fast_connect = true, sim_probe = 'AT+QCCID\r\n', csq_gate = 'AT+QCSQ\r\n', socket_state = 'AT+QISTATE?\r\n' },
    ["EC25"] = { banner = 'EC25', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', preflight = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,%x,0\r\n', setband_all = 'AT+QCFG="band",bff,00b0e18df,0\r\n', fast_connect = true, sim_probe = 'AT+QCCID\r\n', csq_gate = 'AT+QCSQ\r\n', socket_state = 'AT+QISTATE?\r\n' , cipopen_udp_dp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,1\r\n',cipopen_tcp_dp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,1\r\n',qisend = 'AT+QISEND=0,%d\r\n', qcsq_enable = 'AT+QCSQ=1\r\n', }
}

local default_modem = { reset = 'AT+CFUN=1,1\r\r' }
local modem = default_modem

local function option_enabled(option)
    return (P.OPTIONS:get() & option) ~= 0
end

if P.ENABLE:get() == 0 then return end

local uart = serial:find_serial(P.SERPORT:get())
if not uart then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find serial port')
    return
end

local ser_device = serial:find_simulated_device(P.PROTOCOL:get(), P.SCRPORT:get())
if not ser_device then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find SCR_SDEV device')
    return
end

local step = "ATI"
local stats = { bytes_in = 0, bytes_out = 0 }

uart:begin(P.IBAUD:get())

local log_file = io.open('LTE_modem.log', 'w')
local function log_data(s, marker)
    if s and #s > 0 and log_file then
        log_file:write(marker .. '[' .. s .. ']\n')
        log_file:flush()
    end
end

local cs = {
    cops_zero_sent = false, 
    qcsq_tries = 0,
    cipopen_retry = 0, cipopen_sent = false, cipopen_sent_ms = 0,
    hard_reset_strikes = 0, 
    last_step = nil,
    step_timer_ms = 0, step_times = {},
    change_baud = nil, ati_sequence = 0,
    cereg_drop_ms = nil,
    disconnect_ms = nil,  
    last_data_ms = millis(),
    last_CSQ_ms = millis(),
    last_CSQ_reply_ms = uint32_t(0),
    last_parse_ms = uint32_t(0),
    last_route_ms = uint32_t(0),
    last_send_data_ms = uint32_t(0),
    sim_probe_count = 0,
    cpin_probe_n = 0,
    csq_toggle = false,
    post_reset = true,
    cipopen_preclosed = false,
    last_sig_print_ms = nil,
    last_csq_print_ms = nil,
    last_poll_dbg_ms = nil,
    last_parse_dbg_ms = nil,
    direct_push = false,
    qcsq_armed = false,
    rx_need = 0,            -- bytes left to consume in current +QIURC recv block
    tx_state = "idle",      -- idle | prompt | sendok
    tx_pending = "",
    tx_ms = 0,
    consec_stall = 0,       -- consecutive QISEND stalls with no SEND OK between
    dp_closed = false,
    recv_nolen_warned = false,
    ati_dbg_ms = nil,
    dbg_qisend = 0, 
    dbg_prompt = 0, 
    dbg_sendok = 0, 
    dbg_stall = 0,
    dbg_recv = 0,
    last_tx_dbg_ms = nil
}

local function uart_read()
    local s = uart:readstring(512)
    if not s then return "" end
    log_data(s, '<<<')
    stats.bytes_in = stats.bytes_in + #s
    if #s > 0 and step ~= "CONNECTED" then cs.last_data_ms = millis() end 
    return s
end

local buf = { uart="", modem="", fc="", parse="", setup="", at_scan="", dp="" }

local function uart_write_pending()
    if #buf.uart > 0 then
        local n = uart:writestring(buf.uart)
        buf.uart = buf.uart:sub(n+1)
    end
end

local function uart_write(s)
    buf.uart = buf.uart .. s
    if option_enabled(OPT.LOGALL) or step ~= "CONNECTED" then
        log_data(s, '>>>')
    end
    stats.bytes_out = stats.bytes_out + #s
    return #s
end

local cmux = { FLAG = 0xF9, UIH = 0xEF, SABM = 0x2F, EA = 0x01, CR_SEND = 0x02, DLC_AT = 1, DLC_DATA = 2, buffers = {[1] = "", [2] = ""} }

local last_mccmnc = nil
local last_band = nil
local lte_track = { band = nil, cid = nil }

local fcs_table = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75, 0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69, 0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d, 0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51, 0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05, 0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19, 0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d, 0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21, 0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95, 0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89, 0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad, 0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1, 0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5, 0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9, 0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd, 0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1, 0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf
}

local function fcs_calc(data)
    local fcs = 0xff
    for i = 1, #data do
        local byte = string.byte(data, i)
        fcs = fcs_table[((fcs ~ byte) & 0xff) + 1] ~ (fcs >> 8)
    end
    return (~fcs) & 0xff
end

function cmux.encode_cmux_frame(dlc, dtype, data)
    local addr = string.char((dlc << 2) | cmux.EA | cmux.CR_SEND)
    local ctrl = string.char(dtype | 0x10)
    local len = #data
    local len_byte = string.char((len << 1) | cmux.EA)
    local header = addr .. ctrl .. len_byte
    local fcs = string.char(fcs_calc(header))
    return string.char(cmux.FLAG) .. header .. data .. fcs .. string.char(cmux.FLAG)
end

local found_cmux = false
-- Session-sticky flag set when CMUX is known broken on the current modem.
-- Cleared only on full Lua script reload (e.g. on Pixhawk reboot).
-- Set by either:
--   (a) firmware revision auto-detect at modem identification time, OR
--   (b) CPIN failure 5x after CMUX was supposedly set up (broken firmware
--       that accepted AT+CMUX=0 but never opened DLC channels)
local cmux_force_disabled = false
-- Per-attempt marker: true once step_CMUX successfully transitions to BAUD.
-- If CPIN later fails 5 probes while this is true, we conclude CMUX setup
-- was a lie and trigger the (b) fallback above.
local cmux_was_set = false
-- Stores the firmware revision string (e.g. "EC25EFAR08A07M4G") once detected.
-- Used for diagnostics and firmware-based CMUX auto-disable.
local modem_revision = ""

-- Known-broken firmware revision patterns. If the modem reports a revision
-- matching any of these substrings, we skip CMUX entirely (saves the
-- ~25 second broken-CMUX recovery cycle on every boot).
-- Add more patterns here as Quectel ships new broken revisions.
local BROKEN_CMUX_REVISIONS = {
    "EC25EFAR08A07M4G",   -- confirmed broken; SABM frames silently ignored
    -- "EC25EFAR08A08M4G", -- add future broken revisions here
}

local function is_known_broken_revision(rev)
    if not rev or #rev == 0 then return false end
    for _, pattern in ipairs(BROKEN_CMUX_REVISIONS) do
        if rev:find(pattern, 1, true) then return true end
    end
    return false
end

-- Known-good EC25 firmware family (CMUX confirmed working on the bench).
-- Anything that is EC25 but neither known-good nor known-broken is treated as
-- CMUX-capable but flagged with a one-time warning so an unverified revision
-- that turns out to hang is diagnosable instead of silently wrong.
local KNOWN_GOOD_EC25_PREFIX = "EC25EFAR06"

local function cmux_enabled()
    if cmux_force_disabled then return false end
    if found_cmux then return true end
    return modem and modem.cmux and not option_enabled(OPT.NOMUX)
end

-- Direct-push transport is derived from modem state, NOT from an operator bit:
-- used only when CMUX is unavailable AND the current modem table defines
-- direct-push commands (currently EC25 only — no other family has these fields).
-- Result: broken EC25 auto-uses direct push, good EC25 auto-uses CMUX, all other
-- modems keep their normal CMUX path, with one LTE_OPTIONS value for all.
local function want_direct_push()
    return (not cmux_enabled()) and modem ~= nil and modem.cipopen_udp_dp ~= nil
end

local function AT_send(atcmd)
    local s = cmux_enabled() and cmux.encode_cmux_frame(cmux.DLC_AT, cmux.UIH, atcmd) or atcmd
    return uart_write(s) == #s
end

local function send_data_reset()
    if modem.reset then
        AT_send(modem.reset)
        if not modem.reset_not_baudrate then uart:begin(P.IBAUD:get()) end
        found_cmux = false
        gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: sent reset")
        return
    end
end

local function handle_error(s)
    if s and s:find('\nERROR\r\n') then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: error response from modem')
        send_data_reset(); step = "ATI"
        return true
    end
    return false
end

function cmux.send_sabm()
    uart_write(cmux.encode_cmux_frame(0, cmux.SABM, ""))
    uart_write(cmux.encode_cmux_frame(1, cmux.SABM, ""))
    uart_write(cmux.encode_cmux_frame(2, cmux.SABM, ""))
end

function cmux.parse_cmux_frame(buf_in)
    local start_idx = buf_in:find(string.char(cmux.FLAG))
    if not start_idx then return nil, nil, nil end
    if #buf_in < 6 then return nil, nil, nil, "short" end
    local len_byte = buf_in:byte(4)
    if (len_byte & cmux.EA) == 0 then return nil, nil, nil end
    local len = len_byte >> 1
    local end_idx = 6 + len
    if buf_in:byte(end_idx) ~= cmux.FLAG then return nil, nil, nil, "short" end

    local frame = buf_in:sub(start_idx + 1, end_idx - 1)
    if #frame < 4 then return nil, nil, nil end

    local addr = frame:byte(1)
    local ctrl = frame:byte(2)

    if ctrl == cmux.SABM then return nil, nil, buf_in:sub(end_idx + 1) end
    if (ctrl & 0xef) ~= cmux.UIH then return nil, nil, nil end
    if #frame ~= 3 + len + 1 then return nil, nil, nil end

    local data = frame:sub(4, 3 + len)
    local fcs_field = frame:byte(3 + len + 1)
    local header = frame:sub(1, 3)
    if fcs_calc(header) ~= fcs_field then return nil, nil, nil end

    local dlc = (addr >> 2) & 0x3F
    local remainder = buf_in:sub(end_idx + 1)
    return dlc, data, remainder
end

function cmux.feed_uart_in(raw)
    while #raw > 0 do
        local dlc, data, rest, err = cmux.parse_cmux_frame(raw)
        if not dlc or not data or not rest then
            if err == "short" then return raw end
            return ""
        end
        if cmux.buffers[dlc] then cmux.buffers[dlc] = cmux.buffers[dlc] .. data end
        raw = rest
    end
    return raw
end

local function data_send(data)
    local s = cmux_enabled() and cmux.encode_cmux_frame(cmux.DLC_DATA, cmux.UIH, data) or data
    return uart_write(s) == #s
end

local function data_send_connected(data)
    local s = cmux_enabled() and cmux.encode_cmux_frame(cmux.DLC_DATA, cmux.UIH, data) or data
    local n = uart_write(s)
    stats.bytes_out = stats.bytes_out + n
    return n == #s
end

local function reset_buffers()
    cs.last_data_ms = millis()
    buf.modem = ""; buf.fc = ""; buf.parse = ""; buf.setup = ""; buf.at_scan = ""
    cmux.buffers[cmux.DLC_AT] = ""; cmux.buffers[cmux.DLC_DATA] = ""
    while ser_device:available() > 0 do ser_device:readstring(512) end
end

local function reset_state()
    step = "ATI"; modem = default_modem; found_cmux = false
    reset_buffers(); buf.uart = ""
    lte_track.band = nil; lte_track.cid = nil
    cs.cops_zero_sent = false; cs.qcsq_tries = 0
    cs.cipopen_retry = 0; cs.cipopen_sent = false; cs.cipopen_sent_ms = 0
    cs.hard_reset_strikes = 0 
    cs.sim_probe_count = 0; cs.cpin_probe_n = 0
    cs.cereg_drop_ms = nil      
    cs.step_times = {}; cs.step_timer_ms = millis():tofloat()
    cs.post_reset = true
    cs.cipopen_preclosed = false
    cmux_was_set = false  -- per-attempt marker; cmux_force_disabled stays sticky
    cs.ati_dbg_ms = nil
    cs.consec_stall = 0
end

local function reset_to_ATI()
    send_data_reset(); uart_write_pending(); reset_state()
end

-- Extract firmware revision from ATI response and check against broken list.
-- ATI typically returns something like:
--   Quectel\r\nEC25\r\nRevision: EC25EFAR08A07M4G\r\nOK\r\n
local function check_modem_revision(s)
    -- Match "Revision: <something>" pattern (Quectel style)
    local rev = s:match("Revision:%s*([%w%._]+)")
    if not rev then
        -- Fallback: match standalone firmware version string
        rev = s:match("(EC25[%w]+)") or s:match("(EC20[%w]+)") or s:match("(BG95[%w]+)")
    end
    if rev and #rev > 0 then
        modem_revision = rev
        gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: firmware " .. rev)

        -- CMUX auto-disable / direct-push is EC25-specific. Other families
        -- (SimCom, EC20, BG95, Air780…) fall through untouched and use CMUX
        -- normally via their modem.cmux field.
        if rev:find("EC25", 1, true) == 1 then
            if is_known_broken_revision(rev) then
                cmux_force_disabled = true
                gcs:send_text(MAV_SEVERITY.WARNING,
                    "LTE_modem: known-broken CMUX firmware, using direct push")
            elseif rev:find(KNOWN_GOOD_EC25_PREFIX, 1, true) == 1 then
                -- known-good CMUX firmware: normal path, no message needed
            else
                gcs:send_text(MAV_SEVERITY.WARNING,
                    "LTE: untested EC25 fw " .. rev .. " - assuming CMUX OK")
            end
        end
    end
end

local function check_modem_banner(s)
    for model in pairs(modem_list) do
        if s:find(modem_list[model].banner) then
            modem = modem_list[model]
            gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: found modem: " .. model)
            -- Try to extract firmware revision from the same response
            check_modem_revision(s)
            return
        end
    end
end

-- =========================================================================
-- AT COMMAND PARSERS
-- =========================================================================

local function check_CSQ(s)
    local rssi_raw, ber_raw = s:match("%+CSQ:%s*(%d+),(%d+)")
    if rssi_raw then
        gcs:send_named_float('LTE_RSSI', rssi_raw)
        logger:write("LTE",'RSSI,BER,Bin,Bout','iiII', rssi_raw, ber_raw, stats.bytes_in, stats.bytes_out)
        -- DEBUG: print CSQ values to GCS text (throttled to once per 5s)
        if not cs.last_csq_print_ms or (millis() - cs.last_csq_print_ms) > 5000 then
            cs.last_csq_print_ms = millis()
            gcs:send_text(MAV_SEVERITY.INFO,
                string.format("LTE CSQ: RSSI=%s BER=%s", rssi_raw, ber_raw))
        end
        return true
    end
    return false
end

local function check_CGACT(s)
    local ctx, active = s:match("%+CGACT:%s*(%d+),(%d+)")
    if ctx then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("CGACT: %d,%d", tonumber(ctx) or 0, tonumber(active) or 0))
        return true
    end
    return false
end

local function check_CPSI(s)
    if not s:find("+CPSI") then return false end
    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))

    local system_mode, operation_mode, mcc_mnc, tac_str, scell_id_str, pcid_str, earfcn_band, ul_freq_str, dl_freq_str, tdd_cfg_str, rsrq_str, rsrp_str, rssi_str, sinr_str =
    s:match("+CPSI:%s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([%-]?%d+),([%-]?%d+),([%-]?%d+),([%-]?%d+)")

    if system_mode and sinr_str then
        local tac = tonumber(tac_str:match("0x(%w+)"), 16) or tonumber(tac_str) or 0
        local scell_id = tonumber(scell_id_str) or 0
        local band = earfcn_band:match("[^%d]+(%d+)") or -1
        logger:write("LTES",'Md,Op,MCC,TAC,CID,PID,BND,F,DF,TDD,RP,RQ,RS,SR','NNNIIINHhhhhhh',
                     system_mode, operation_mode, mcc_mnc, tac, scell_id, tonumber(pcid_str) or 0, earfcn_band,
                     tonumber(ul_freq_str) or 0, tonumber(dl_freq_str) or 0, tonumber(tdd_cfg_str) or 0, tonumber(rsrp_str) or 0, tonumber(rsrq_str) or 0, tonumber(rssi_str) or 0, tonumber(sinr_str) or 0)
        if option_enabled(OPT.SIGNALS) then
            gcs:send_named_float('LTE_RSRP', tonumber(rsrp_str) or 0)
            gcs:send_named_float('LTE_RSRQ', tonumber(rsrq_str) or 0)
            gcs:send_named_float('LTE_BAND', band)
            gcs:send_named_float('LTE_CID', scell_id>>8)
            local mcc, mnc = mcc_mnc:match("(%d+)-(%d+)")
            if mcc and mnc then gcs:send_named_float('LTE_MCCMNC', mcc*100+mnc) end
        end

        local band_num = tonumber(band) or -1
        local tower_id = scell_id >> 8
        if lte_track.band == nil then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE: connected on Band %s (%s)", tostring(band), earfcn_band))
            lte_track.band = band_num
        end
        if lte_track.cid == nil then lte_track.cid = tower_id end
        if lte_track.band ~= nil and band_num ~= lte_track.band then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("LTE WARNING: band switch %d -> %d (%s)", lte_track.band, band_num, earfcn_band))
            lte_track.band = band_num
        end
        if lte_track.cid ~= nil and tower_id ~= lte_track.cid then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("LTE WARNING: cell tower switch CID %d -> %d", lte_track.cid, tower_id))
            lte_track.cid = tower_id
        end
        return true
    end
    return false
end

local function check_QENG(s)
    if not s:find('+QENG:%s*"servingcell"') then return false end
    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))
    local data_str = s:match('%+QENG:%s*"servingcell",(.*)')
    if not data_str then return false end
    data_str = data_str:gsub('"', '')

    local t = {}
    for part in string.gmatch(data_str .. ",", "(.-),") do table.insert(t, part) end

    if t[2] == "LTE" or t[2] == "eMTC" then
        local is_fdd = (t[3] == "FDD")
        local tac_idx = is_fdd and 12 or 11

        local mcc = tonumber(t[4]) or 0
        local mnc = tonumber(t[5]) or 0
        local cid = tonumber(t[6], 16) or 0
        local earfcn = tonumber(t[8]) or 0
        local band = tonumber(t[9]) or -1
        local tac = tonumber(t[tac_idx], 16) or 0
        local rsrp = tonumber(t[tac_idx+1]) or 0
        local rsrq = tonumber(t[tac_idx+2]) or 0
        local rssi = tonumber(t[tac_idx+3]) or 0
        local sinr = tonumber(t[tac_idx+4]) or 0

        logger:write("LTES", 'MCC,MNC,TAC,CID,PID,EF,RSRP,RSRQ,RSSI,SINR', 'iiiiiiiiii', mcc, mnc, tac, cid, tonumber(t[7]) or 0, earfcn, rsrp, rsrq, rssi, sinr)

        -- DEBUG: always send named floats (regardless of OPT.SIGNALS) for testing.
        -- Revert this gating once you've confirmed values are reaching MP.
        gcs:send_named_float('LTE_RSRP', rsrp)
        gcs:send_named_float('LTE_RSRQ', rsrq)
        gcs:send_named_float('LTE_SINR', sinr)
        gcs:send_named_float('LTE_BAND', band)
        gcs:send_named_float('LTE_CID', cid>>8)
        gcs:send_named_float('LTE_MCCMNC', mcc*100+mnc)

        local tower_id = cid >> 8
        if lte_track.band == nil then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE: connected on Band %d (EARFCN %d)", band, earfcn))
            lte_track.band = band
        end
        if lte_track.cid == nil then lte_track.cid = tower_id end

        if lte_track.band ~= nil and band ~= lte_track.band then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("LTE WARNING: band switch %d -> %d (EARFCN %d)", lte_track.band, band, earfcn))
            lte_track.band = band
        end
        if lte_track.cid ~= nil and tower_id ~= lte_track.cid then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("LTE WARNING: cell tower switch CID %d -> %d", lte_track.cid, tower_id))
            lte_track.cid = tower_id
        end
        return true
    end
    return false
end

local function check_QCSQ(s)
    -- +QCSQ: "<sysmode>",<rssi>,<rsrp>,<sinr>,<rsrq>
    local mode, rssi, rsrp, sinr_raw, rsrq =
        s:match('+QCSQ:%s*"([^"]+)",(%-?%d+),(%-?%d+),(%-?%d+),(%-?%d+)')
    if not mode then return false end
    rssi = tonumber(rssi); rsrp = tonumber(rsrp)
    sinr_raw = tonumber(sinr_raw); rsrq = tonumber(rsrq)
    logger:write("LTEQ", 'RSSI,RSRP,SINRr,RSRQ', 'iiii', rssi, rsrp, sinr_raw, rsrq)
    -- RSSI/RSRP/RSRQ are direct dBm/dB — publish continuously.
    gcs:send_named_float('LTE_RSSI', rssi)
    gcs:send_named_float('LTE_RSRP', rsrp)
    gcs:send_named_float('LTE_RSRQ', rsrq)
    -- QCSQ <sinr> is a SCALED index (not dB). Deliberately NOT published as
    -- LTE_SINR — the slow QENG poll owns that with a calibrated dB value.
    -- SINRr is logged above; once you find the linear map to QENG dB you can
    -- publish it here for continuous SINR too.
    return true
end


local function handle_AT_reply(s)
    check_CSQ(s)
    if check_CPSI(s) then return end
    if check_QENG(s) then return end
    if check_CGACT(s) then return end
    if s:find("PPPD: DISCONNECTED") then step = "PPPOPEN" end
end

local dp = {}

-- Receive parser. buf.dp carries AT-mode bytes interleaving:
--   +QIURC: "recv",<id>,<len>\r\n<len raw bytes>  -> downlink payload (opaque)
--   +QCSQ: ...                                     -> signal URC
--   +CEREG: ...                                    -> registration URC
--   +QIURC: "closed",<id>                          -> socket dropped
--   \r\n>  / SEND OK / SEND FAIL                    -> QISEND handshake
-- Raw payload inside a recv block is consumed by exact byte count, since it
-- may contain \r\n or bytes that look like URCs.
function dp.process_rx(now_ms)
    while true do
        if cs.rx_need > 0 then
            if #buf.dp == 0 then return end
            local take = cs.rx_need
            if take > #buf.dp then take = #buf.dp end
            buf.fc = buf.fc .. buf.dp:sub(1, take)
            buf.dp = buf.dp:sub(take + 1)
            cs.rx_need = cs.rx_need - take
            cs.last_data_ms = now_ms
            if cs.rx_need > 0 then return end
        end

        local nl = buf.dp:find("\r\n", 1, true)
        if not nl then
            -- The only non-line token we care about is the QISEND prompt
            -- "\r\n> " (no trailing CRLF), and only while waiting for it.
            if cs.tx_state == "prompt" then
                local gt = buf.dp:find("> ", 1, true)   -- Quectel prompt "\r\n> "; CRLF already consumed by line loop
                if gt then
                    buf.dp = buf.dp:sub(gt + 2)
                    uart_write(cs.tx_pending)
                    cs.tx_state = "sendok";
                    cs.dbg_prompt = cs.dbg_prompt + 1
                    cs.tx_ms = now_ms:tofloat()
                else
                    if #buf.dp > 4096 then buf.dp = buf.dp:sub(-2048) end
                    return
                end
            else
                if #buf.dp > 4096 then buf.dp = buf.dp:sub(-2048) end
                return
            end
        else
            local line = buf.dp:sub(1, nl - 1)
            buf.dp = buf.dp:sub(nl + 2)
            if line == "" then
                -- skip blank line
            elseif line:find('"recv"', 1, true) then
                local len = line:match('"recv",%s*%d+,%s*(%d+)')   -- direct-push form: connID,length
                if len then
                    cs.rx_need = tonumber(len)
                else
                    if not cs.recv_nolen_warned then
                        cs.recv_nolen_warned = true
                        gcs:send_text(MAV_SEVERITY.WARNING, 'LTE: recv URC without length - check QICFG recvind')
                    end
                end
                cs.dbg_recv = cs.dbg_recv + 1
            elseif line:find('+QCSQ:', 1, true) then
                check_QCSQ(line)
            elseif line:find('"closed"', 1, true) then
                cs.dp_closed = true
            elseif line:find('SEND OK', 1, true) then
                if cs.tx_state == "sendok" then cs.tx_state = "idle"; cs.tx_pending = ""; cs.dbg_sendok = cs.dbg_sendok + 1 end
                cs.last_data_ms = now_ms   -- uplink success proves the link is alive
            elseif line:find('SEND FAIL', 1, true) then
                if cs.tx_state == "sendok" then
                    if #cs.tx_pending > 0 then buf.modem = cs.tx_pending .. buf.modem end  -- requeue for retry
                    cs.tx_state = "idle"; cs.tx_pending = ""
                end
            elseif line:find('+CEREG:', 1, true) or line:find('+CREG:', 1, true) then
                if line:find('+CEREG: 0') or line:find('+CEREG: 2') or
                   line:find('+CREG: 0,0') or line:find('+CREG: 0,2') then
                    if not cs.cereg_drop_ms then
                        cs.cereg_drop_ms = now_ms
                        gcs:send_text(MAV_SEVERITY.WARNING, string.format('LTE: CEREG lost — %ds grace', P.GRACE:get()))
                    end
                elseif line:find('+CEREG: 1') or line:find('+CEREG: 5') or
                       line:find('+CREG: 0,1') or line:find('+CREG: 0,5') then
                    cs.cereg_drop_ms = nil
                end
            elseif line:find('+QENG:', 1, true) or line:find('+CPSI:', 1, true) or line:find('+CSQ:', 1, true) then
                handle_AT_reply(line)
            elseif line:find('ERROR', 1, true) then
                if cs.tx_state ~= "idle" then cs.tx_state = "idle"; cs.tx_pending = "" end
            end
            -- else: echo / OK / other -> ignore
        end
    end
end

-- Transmit pump. One QISEND in flight at a time, non-blocking across ticks.
function dp.process_tx(now_ms, budget)

    if cs.tx_state ~= "idle" then
        if (now_ms:tofloat() - cs.tx_ms) > 2000 then
            -- Requeue the unconfirmed chunk rather than dropping it.
            if #cs.tx_pending > 0 then buf.modem = cs.tx_pending .. buf.modem end
            cs.tx_state = "idle"; cs.tx_pending = ""; cs.dbg_stall = cs.dbg_stall + 1
            cs.consec_stall = cs.consec_stall + 1

            local dead_n = math.floor(P.TX_DEAD:get())
            if dead_n > 0 and cs.consec_stall >= dead_n then
                -- N stalls, zero SEND OKs in between: bearer silently died under
                -- the UDP socket (post-handover). No URC will ever fire for this.
                -- Go straight to the recovery path that provably works:
                -- QISTATE? -> QICLOSE -> QIOPEN (~150ms), skipping blind reopens
                -- that fail while the modem still thinks socket 0 is in use.
                gcs:send_text(MAV_SEVERITY.ERROR,
                    string.format('LTE: %d consecutive stalls - socket dead, fast recover', cs.consec_stall))
                cs.consec_stall = 0
                if not cs.disconnect_ms then
                    cs.disconnect_ms = now_ms:tofloat() - (dead_n * 2000)
                end
                cs.last_data_ms = now_ms
                cs.cipopen_sent = false
                step = modem.socket_state and "SOCKET_STATE" or "CIPOPEN"
                return
            end
            gcs:send_text(MAV_SEVERITY.WARNING,
                string.format('LTE: QISEND stalled %d/%d', cs.consec_stall, math.max(1, math.floor(P.TX_DEAD:get()))))
        end
        return
    end
    if #buf.modem == 0 then return end
    if budget ~= nil and budget <= 0 then return end   -- rate quota exhausted this tick
    local n = #buf.modem
    if n > 1024 then n = 1024 end
    if budget ~= nil and n > budget then n = budget end
    cs.tx_pending = buf.modem:sub(1, n)
    buf.modem = buf.modem:sub(n + 1)   -- popped into tx_pending; requeued on stall/FAIL, cleared on SEND OK
    cs.last_send_data_ms = now_ms
    AT_send(string.format(modem.qisend, n))
    cs.dbg_qisend = cs.dbg_qisend + 1
    cs.tx_state = "prompt"; cs.tx_ms = now_ms:tofloat()
end


-- =========================================================================
-- MAIN STATE MACHINE STEPS
-- =========================================================================

local function step_ATI()
    local s = uart_read()
    -- Stay silent on a normal fast boot (ATI exits in ~3s), but once ATI drags
    -- past 2s (modem mid-reboot after a reset), chirp every 5s so the recovery
    -- window is no longer a black hole in the messages tab.
    local ati_s = (millis():tofloat() - cs.step_timer_ms) / 1000
    if ati_s > 2 and (not cs.ati_dbg_ms or (millis() - cs.ati_dbg_ms) > 5000) then
        cs.ati_dbg_ms = millis()
        gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE: waiting for modem (ATI, %ds)', math.floor(ati_s)))
    end
    if s and modem == default_modem then check_modem_banner(s) end
    if modem ~= default_modem then
        if not cmux_enabled() then step = "BAUD" else step = "CMUX" end
        return
    end
    if not option_enabled(OPT.NOMUX) and not cmux_force_disabled and s and #s >= 4 and s:byte(1) == cmux.FLAG and s:byte(-1) == cmux.FLAG then
        found_cmux = true; gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: in CMUX mode"); log_data("{INCMUX}", '***')
        AT_send('ATI\r'); return
    end
    if cs.ati_sequence % 3 == 2 then uart_write('+++')
    elseif cs.ati_sequence % 3 == 1 and not option_enabled(OPT.NOMUX) then uart_write(cmux.encode_cmux_frame(cmux.DLC_AT, cmux.UIH, "ATI\r"))
    else uart_write('\rATI\r') end

    if cs.ati_sequence % 10 == 5 then uart:begin(P.BAUD:get()); log_data(string.format("{BAUD=%d}", P.BAUD:get()), '***') end
    if cs.ati_sequence % 10 == 9 then uart:begin(P.IBAUD:get()); log_data(string.format("{BAUD=%d}", P.IBAUD:get()), '***') end
    cs.ati_sequence = cs.ati_sequence + 1
end

local function step_BAUD()
    if modem.setbaud and P.BAUD:get() ~= P.IBAUD:get() then
        cs.change_baud = P.BAUD:get()
        AT_send(string.format(modem.setbaud, cs.change_baud))
    end
    step = "CPIN"
end

local function set_MCCMNC()
    if not modem.mccmnc then return end
    local mccmnc = math.floor(P.MCCMNC:get())
    if mccmnc > 0 then AT_send(string.format(modem.mccmnc, mccmnc))
    elseif mccmnc == 0 then AT_send("AT+COPS=0\r\n") end
    last_mccmnc = mccmnc
end

local function set_BAND()
    if not modem.setband and not modem.setband_mask then return end
    local band = math.floor(P.BAND:get())
    if band > 0 then
       if modem.setband_mask then AT_send(string.format(modem.setband_mask, 1<<(band-1)))
       else AT_send(string.format(modem.setband, band)) end
    elseif band == 0 then AT_send(modem.setband_all) end
    last_band = band
end

local function step_CONFIG()
    set_BAND(); set_MCCMNC()
    if modem.config_extra then AT_send(modem.config_extra) end
    if modem.fast_connect then AT_send('AT+CEREG=1\r\n') end
    step = "CREG"
end

local function step_CPIN()
    local s = uart_read()
    if s and s:find("READY") then step = "CONFIG"; return end
    if s and (s:find("%+QCCID:") or s:find("%+ICCID:")) then
        cs.sim_probe_count = (cs.sim_probe_count or 0) + 1
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "LTE: SIM hardware OK but unready (%d/3)", cs.sim_probe_count))
        if cs.sim_probe_count >= 3 then
            gcs:send_text(MAV_SEVERITY.CRITICAL,
                "LTE FATAL: SIM unresponsive after 3 probes. Halting.")
            step = "HALT"; return
        end
    elseif s and (s:find("ERROR: 10") or s:find("NOT INSERTED") or s:find("SIM not inserted")) then
        gcs:send_text(MAV_SEVERITY.CRITICAL, "LTE FATAL: SIM CARD MISSING! Halting sequence.")
        step = "HALT"; return
    end

    local now_ms = millis():tofloat()
    local time_in_step = (now_ms - cs.step_timer_ms) / 1000

    -- Phase 1: first 1s — normal polling for +CPIN: READY (reduced from 2s)
    if time_in_step < 1.0 then
        AT_send('AT+CPIN?\r\n')
        return
    end

    -- Phase 2: 1–5s — probe once per second (3 probes max), then hard reset
    local probe_tick = math.floor(time_in_step) -- integer seconds since step start
    if probe_tick > cs.cpin_probe_n then
        cs.cpin_probe_n = probe_tick
        local probe_num = probe_tick - 1  -- 1,2,3 for seconds 2,3,4
        if probe_num <= 5 then
            gcs:send_text(MAV_SEVERITY.WARNING,
                string.format("LTE: SIM not ready, probe %d/5", probe_num))
            AT_send('AT+CPIN?\r\n')
            if modem.sim_probe then AT_send(modem.sim_probe) end
        else
            -- Defense in depth: if CMUX was set up but CPIN still fails after 5
            -- probes, the firmware is broken even though revision auto-detect
            -- didn't catch it. Add this revision to BROKEN_CMUX_REVISIONS so
            -- future boots will catch it via auto-detect.
            local is_known_good_fw = modem_revision:find(KNOWN_GOOD_EC25_PREFIX, 1, true) == 1
            if cmux_was_set and not cmux_force_disabled and not is_known_good_fw then
                gcs:send_text(MAV_SEVERITY.WARNING,
                    "LTE: CPIN failed after CMUX setup — disabling CMUX (firmware bug)")
                if #modem_revision > 0 then
                    gcs:send_text(MAV_SEVERITY.WARNING,
                        "LTE: consider adding '" .. modem_revision .. "' to BROKEN_CMUX_REVISIONS")
                end
                cmux_force_disabled = true
                -- Send a framed reset so the modem actually receives it
                -- (its UART is in CMUX framing mode, plain bytes are dropped)
                if modem.reset then
                    uart_write(cmux.encode_cmux_frame(cmux.DLC_AT, cmux.UIH, modem.reset))
                    uart_write_pending()
                end
            end
            gcs:send_text(MAV_SEVERITY.CRITICAL, "LTE: CPIN failed 5 probes, hard reset")
            reset_to_ATI()
        end
    end
end

local function step_QENG()
    local s = uart_read()
    if s and s:find("+QENG") then
        handle_AT_reply(s)
        step = modem.socket_state and "SOCKET_STATE" or (modem.cipclose and "CIPCLOSE" or "CIPOPEN")
        return
    end
    AT_send(modem.preflight)
end

local function step_CREG()
    local raw = uart_read()
    if raw and #raw > 0 then buf.setup = buf.setup .. raw end
    if #buf.setup > 4096 then buf.setup = "" end 
    
    local s = buf.setup

    if handle_error(s) then 
        buf.setup = "" 
        return 
    end

    if s and #s > 0 then
        -- Use #raw to ensure we only evaluate CMUX if new data just arrived
        if cmux_enabled() and #raw > 0 and #s > 4 and not cmux.parse_cmux_frame(s) then 
            -- Check if it's just a standard AT text response before panicking
            if not (s:find('+CEREG:') or s:find('+CREG:') or s:find('OK\r\n')) then
                step = "CMUX"; buf.setup = ""; return 
            end
        end
        
        local reg = s:match('+CEREG: %d,(%d+)') or s:match('+CREG: %d,(%d+)\r\n')
        
        if reg == "1" or reg == "5" then
            buf.setup = "" -- Clear after success
            cs.cops_zero_sent = false
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CREG OK')
            if P.PROTOCOL:get() == PPP then
                step = modem.cgact and "CGACT" or "PPPOPEN"
            else
                if modem.fast_connect then step = "SIGNAL_GATE"
                elseif modem.preflight then step = "QENG"
                elseif modem.cipmode then step = "CIPMODE"
                elseif modem.cipclose then step = "CIPCLOSE"
                else step = "CIPOPEN" end
            end
            return
            
        elseif reg == "0" or reg == "3" then
            buf.setup = "" 
            if modem.fast_connect then
                local time_in_step = (millis():tofloat() - cs.step_timer_ms) / 1000
                if time_in_step >= 2.0 then
                    if not cs.cops_zero_sent then
                        gcs:send_text(MAV_SEVERITY.WARNING, 'LTE CREG: not registered, auto-selecting (COPS=0)')
                        AT_send('AT+COPS=0\r\n')
                        cs.cops_zero_sent = true
                    elseif time_in_step > 12.0 then
                        gcs:send_text(MAV_SEVERITY.CRITICAL, 'LTE: CREG search timed out, hard reset')
                        reset_to_ATI(); return
                    end
                end
            else
                gcs:send_text(MAV_SEVERITY.WARNING, 'LTE CREG: not registered')
                AT_send("AT+CFUN=1\r\n"); AT_send("AT+COPS?\r\n")
            end
            return
            
        elseif reg == "2" then
            -- Do not clear the buffer here! Let it print, and let the OK catch it below.
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CREG searching...')
        end

        -- Self-cleaning mechanism: wipe the slate clean when the modem finishes a sentence
        if s:find('OK\r\n') or s:find('ERROR\r\n') then
            buf.setup = ""
        end
    end
    
    -- Unconditionally poll the modem (the 200ms delay in run_step prevents spamming)
    AT_send(modem.fast_connect and 'AT+CEREG?\r\n' or 'AT+CREG?\r\n')
end

local function step_SOCKET_STATE()
    local s = uart_read()
    if s and (s:find('+QISTATE:') or s:find('+CIPOPEN:') or s:find('\r\nOK\r\n') or s:find('\nERROR\r\n')) then
        if s:find('+QISTATE: 0') or s:find('+CIPOPEN: 0,"TCP"') or s:find('+CIPOPEN: 0,"UDP"') then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: socket open, closing first')
            step = "CIPCLOSE"
        else
            step = "CIPOPEN"
        end
        return
    end
    if modem.socket_state then AT_send(modem.socket_state) else step = "CIPOPEN" end
end

local function step_CGACT()
    local s = uart_read()
    if handle_error(s) then return end
    if s and s:find('\r\nOK\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CGACT OK')
        if P.PROTOCOL:get() == PPP then step = "PPPOPEN"; cs.last_data_ms = millis() else step = "CIPMODE" end
        return
    end
    data_send(modem.cgact)
    if modem.cfun then data_send(modem.cfun) end
end

local function step_CMUX()
    if found_cmux then
        cmux.send_sabm()
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CMUX mode set')
        cmux_was_set = true  -- marker for CPIN-failure fallback
        step = "BAUD"
        return
    end
    local s = uart_read()
    if s then
        if s:find("CME ERROR") then AT_send('AT+CFUN=1\r\n')
        elseif #s >= 4 and (cmux.parse_cmux_frame(s) or s:find('CMUX=0\r\r\nOK\r') or s == string.char(cmux.FLAG,cmux.FLAG,cmux.FLAG,cmux.FLAG)) then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CMUX mode set')
            cmux_was_set = true  -- marker for CPIN-failure fallback
            cmux.send_sabm(); step = "BAUD"; return
        end
    end
    uart_write(modem.cmux)
end

local function step_PPPOPEN()
    local s = uart_read()
    if s and modem.cgact and s:find("\r\nNO CARRIER\r\n") then send_data_reset(); step = "ATI"; return end
    if s and s:find("CME ERROR:") then send_data_reset(); step = "ATI"; return end
    if handle_error(s) then return end
    if s and s:find('CONNECT') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
        reset_buffers(); step = "CONNECTED"; return
    end
    data_send(modem.pppopen)
end

local function step_CIPCLOSE()
    local s = uart_read()
    if s and (s:find('\r\nOK\r\n') or s:find('\nERROR\r\n') or s:find('+QICLOSE') or s:find('+CIPCLOSE')) then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: socket closed, opening')
        step = "CIPOPEN"; return
    end
    AT_send(modem.cipclose)
end

local function step_SIGNAL_GATE()
    if cs.post_reset then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE: Skipping signal gate post-reset')
        if P.PROTOCOL:get() == PPP then
            step = modem.cgact and "CGACT" or "PPPOPEN"
        elseif modem.cipmode then step = "CIPMODE"
        elseif modem.preflight then step = "QENG"
        else step = "SOCKET_STATE" end
        return
    end

    local s = uart_read()
    local rsrp = nil
    if s and s:find('+QCSQ') then
        local rsrp_str = s:match('+QCSQ:%s*"[^"]+",%-?%d+,(%-?%d+)')
        rsrp = tonumber(rsrp_str)
    elseif s and s:find('+CPSI:') then
        local _, _, _, _, _, _, _, _, _, _, _, rsrp_str = s:match("+CPSI:%s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([%-]?%d+),([%-]?%d+)")
        rsrp = tonumber(rsrp_str)
        if rsrp and rsrp < -200 then rsrp = math.floor(rsrp / 10) end
    end

    if rsrp then
        gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE RSRP=%d dBm', rsrp))
        if rsrp >= -110 then
            cs.qcsq_tries = 0
            if P.PROTOCOL:get() == PPP then
                step = modem.cgact and "CGACT" or "PPPOPEN"
            elseif modem.cipmode then step = "CIPMODE"
            elseif modem.preflight then step = "QENG"
            else step = "SOCKET_STATE" end
        else
            cs.qcsq_tries = cs.qcsq_tries + 1
            if cs.qcsq_tries <= 3 then
                gcs:send_text(MAV_SEVERITY.WARNING, string.format('LTE weak RSRP=%d re-scan %d/3', rsrp, cs.qcsq_tries))
                step = "CREG"
            else
                cs.qcsq_tries = 0
                gcs:send_text(MAV_SEVERITY.WARNING, 'LTE weak signal — connecting anyway')
                if modem.cipmode then step = "CIPMODE"
                elseif modem.preflight then step = "QENG"
                else step = "SOCKET_STATE" end
            end
        end
        return
    end
    if modem.csq_gate then AT_send(modem.csq_gate)
    else
        if modem.cipmode then step = "CIPMODE"
        elseif modem.preflight then step = "QENG"
        else step = "SOCKET_STATE" end
    end
end

local function step_CIPMODE()
    local s = uart_read()
    if s:find('AT+CACID=0,0') then gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network context set'); step = "NETOPEN"; return end
    if handle_error(s) then return end
    if s:find('\r\r\nOK\r') or s:find('\r\nOK\r\n') then gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: transparent mode set'); step = "NETOPEN"; return end
    data_send(modem.cipmode)
end

local function step_NETOPEN()
    if not modem.netopen then
        if modem.preflight then step = "QENG"
        elseif modem.fast_connect then step = "SOCKET_STATE"
        else step = "CIPOPEN" end
        return
    end
    local s = uart_read()
    if s:find("AT+CNACT=0,1") and s:find("ERROR") and modem.netclose then data_send(modem.netclose); return end
    if handle_error(s) then return end
    if s and (s:find('NETOPEN\r') or s:find('ACTIVE\r') or s:find('OK\r')) then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network opened')
        if modem.preflight then step = "QENG"
        elseif modem.fast_connect then step = "SOCKET_STATE"
        else step = "CIPOPEN" end
        return
    end
    data_send(modem.netopen)
end

local function step_CIPOPEN()
    local raw = uart_read()

    if cs.last_step == "CIPCLOSE" or cs.last_step == "SOCKET_STATE" then
        cs.cipopen_preclosed = true
    end

    if raw and #raw > 0 then buf.setup = buf.setup .. raw end
    
    -- Safety valve: Prevent infinite RAM growth if modem spams junk
    if #buf.setup > 4096 then buf.setup = "" end 
    
    local s = buf.setup

    -- Pre-close stale socket on first entry only
    if s == "" and modem.cipclose and not cs.cipopen_sent and not cs.cipopen_preclosed then
        gcs:send_text(MAV_SEVERITY.INFO, "LTE: pre-close stale socket")
        cs.cipopen_preclosed = true
        step = "CIPCLOSE"
        return
    end
    if s and #s > 0 then
        if s:find('+CAOPEN: 0,0') and s:find('OK\r\n') and modem.caswitch then 
            data_send(modem.caswitch)
            buf.setup = "" -- Clear after success
            return 
        end
        if s:find('CONNECT') or s:find('+QIOPEN: 0,0') or s:find('+CIPOPEN: 0,0') or (s:find('+CAOPEN: 0,0') and s:find('OK\r\n')) then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
            cs.cipopen_sent = false; cs.cipopen_retry = 0
            cs.hard_reset_strikes = 0 
            cs.consec_stall = 0
            cs.cipopen_preclosed = false
            reset_buffers(); step = "CONNECTED"; return
        end
    end

    local is_error = s and s:find('\nERROR\r\n')
    if is_error then buf.setup = "" end -- Clear buffer on error so we can retry cleanly

    if cs.cipopen_sent then
        local wait = cs.cipopen_sent_ms > 0 and millis():tofloat() - cs.cipopen_sent_ms or 0
        if wait > 3000 or is_error then
            cs.cipopen_retry = cs.cipopen_retry + 1; cs.cipopen_sent = false
            buf.setup = "" -- Clear buffer on timeout
            if cs.cipopen_retry <= 3 then
                gcs:send_text(MAV_SEVERITY.WARNING, string.format('LTE CIPOPEN timeout/error retry %d/3', cs.cipopen_retry))
                if modem.cipclose then step = "CIPCLOSE" end
            else
                cs.hard_reset_strikes = cs.hard_reset_strikes + 1
                if cs.hard_reset_strikes >= 3 then
                    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE CIPOPEN completely dead — HARD RESET')
                    cs.hard_reset_strikes = 0
                    reset_to_ATI()
                else
                    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE CIPOPEN failed 3x — soft reset to CREG')
                    cs.cipopen_retry = 0
                    cs.cipopen_preclosed = false
                    step = "CREG"
                end
            end
        end
        return
    end

    if is_error then return end

    if P.SERVER_PORT:get() <= 0 then gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT"); return end

    local use_dp = want_direct_push()
    local cipopen, dp_used
    if option_enabled(OPT.TCP) then
        if use_dp and modem.cipopen_tcp_dp then
            cipopen = modem.cipopen_tcp_dp; dp_used = true
        else cipopen = modem.cipopen_tcp; dp_used = false end
    else
        if use_dp and modem.cipopen_udp_dp then
            cipopen = modem.cipopen_udp_dp; dp_used = true
        else cipopen = modem.cipopen_udp; dp_used = false end
    end
    cs.direct_push = dp_used
    if dp_used then cs.qcsq_armed = false; cs.tx_state = "idle"; cs.tx_pending = ""; cs.rx_need = 0; buf.dp = "" end
    if not cipopen then
        gcs:send_text(MAV_SEVERITY.ERROR, "LTE: modem has no socket-open command")
        step = "HALT"
        return
    end
    data_send(string.format(cipopen, P.SERVER_IP0:get(), P.SERVER_IP1:get(), P.SERVER_IP2:get(), P.SERVER_IP3:get(), P.SERVER_PORT:get()))
    cs.cipopen_sent = true; cs.cipopen_sent_ms = millis():tofloat()
end

local function step_CONNECTED()
    local s = uart:readstring(512) or ""
    stats.bytes_in = stats.bytes_in + #s
    if option_enabled(OPT.LOGALL) then log_data(s, '<<<') end

    -- Transparent/PPP socket-close strings. In direct push these arrive as
    -- +QIURC: "closed" lines, handled inside dp.process_rx instead.
    if not cs.direct_push then
        if s:find('\r\nCLOSED\r\n') then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connection closed, reconnecting')
            cs.cipopen_sent = false; step = "CIPOPEN"; return
        end
        if s:find('PPPD: DISCONNECTED\r\n') then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: PPP closed, reconnecting')
            step = "PPPOPEN"; return
        end
    end

    local now_ms = millis()

    -- One-shot: arm continuous +QCSQ URC after a direct-push socket opens.
    if cs.direct_push and not cs.qcsq_armed and cs.tx_state == "idle" and modem.qcsq_enable then
        AT_send(modem.qcsq_enable); cs.qcsq_armed = true
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE: enabled +QCSQ push (direct mode)')
    end

    if s and #s > 0 then
        if cmux_enabled() then
            buf.parse = buf.parse .. s
            buf.parse = cmux.feed_uart_in(buf.parse)
            if now_ms - cs.last_parse_ms > 1000 then buf.parse = "" end
            if #cmux.buffers[cmux.DLC_AT] > 0 then
                local at_text = cmux.buffers[cmux.DLC_AT]
                cmux.buffers[cmux.DLC_AT] = ""; cs.last_parse_ms = now_ms
                if at_text:find('+CEREG: 0') or at_text:find('+CEREG: 2') or
                   at_text:find('+CREG: 0,0') or at_text:find('+CREG: 0,2') then
                    if not cs.cereg_drop_ms then
                        cs.cereg_drop_ms = now_ms
                        gcs:send_text(MAV_SEVERITY.WARNING, string.format('LTE: CEREG lost — %ds grace', P.GRACE:get()))
                    end
                elseif at_text:find('+CEREG: 1') or at_text:find('+CEREG: 5') or
                       at_text:find('+CREG: 0,1') or at_text:find('+CREG: 0,5') then
                    cs.cereg_drop_ms = nil
                end
                handle_AT_reply(at_text)
            end
            if #cmux.buffers[cmux.DLC_DATA] > 0 then
                cs.last_data_ms = now_ms; cs.last_parse_ms = now_ms
                buf.fc = buf.fc .. cmux.buffers[cmux.DLC_DATA]; cmux.buffers[cmux.DLC_DATA] = ""
            end

        elseif cs.direct_push then
            buf.dp = buf.dp .. s
            dp.process_rx(now_ms)
            if cs.dp_closed then
                cs.dp_closed = false
                gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: socket closed (URC), reconnecting')
                cs.cipopen_sent = false; cs.tx_state = "idle"; cs.tx_pending = ""
                step = "CIPOPEN"; return
            end

        else
            -- plain transparent no-CMUX (unchanged behaviour)
            buf.fc = buf.fc .. s; cs.last_data_ms = now_ms
            buf.at_scan = (buf.at_scan or "") .. s
            if #buf.at_scan > 4096 then buf.at_scan = buf.at_scan:sub(-2048) end
            if buf.at_scan:find('OK\r\n') or buf.at_scan:find('ERROR\r\n') then
                handle_AT_reply(buf.at_scan)
                local after_ok = buf.at_scan:find('OK\r\n', 1, true)
                local after_err = buf.at_scan:find('ERROR\r\n', 1, true)
                if after_ok then buf.at_scan = buf.at_scan:sub(after_ok + 4)
                elseif after_err then buf.at_scan = buf.at_scan:sub(after_err + 7) end
            end
            if s:find('+CEREG: 0') or s:find('+CEREG: 2') or s:find('+CREG: 0,0') or s:find('+CREG: 0,2') then
                if not cs.cereg_drop_ms then
                    cs.cereg_drop_ms = now_ms
                    gcs:send_text(MAV_SEVERITY.WARNING, string.format('LTE: CEREG lost — %ds grace', P.GRACE:get()))
                end
            elseif s:find('+CEREG: 1') or s:find('+CEREG: 5') or s:find('+CREG: 0,1') or s:find('+CREG: 0,5') then
                cs.cereg_drop_ms = nil
            end
        end

    elseif P.TIMEOUT:get() > 0 and now_ms - cs.last_data_ms > uint32_t(P.TIMEOUT:get() * 1000) then
        if cs.direct_push then
            -- Modem is registered and healthy; only the socket/return-path died.
            -- Reopen the socket (~1-2s) instead of a full AT+CFUN=1,1 reboot
            -- (~120s cold re-registration). Mirrors the CLOSED-URC reconnect path.
            gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: data timeout - reopening socket')
            if not cs.disconnect_ms then cs.disconnect_ms = millis():tofloat() - (P.TIMEOUT:get() * 1000) end
            cs.last_data_ms = now_ms
            cs.cipopen_sent = false
            cs.tx_state = "idle"; cs.tx_pending = ""
            cs.consec_stall = 0
            step = modem.socket_state and "SOCKET_STATE" or "CIPOPEN"; return
        else
            gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: data timeout')
            if not cs.disconnect_ms then cs.disconnect_ms = millis():tofloat() - (P.TIMEOUT:get() * 1000) end
            reset_to_ATI(); return
        end
    end

    local grace_ms = P.GRACE:get() * 1000
    if cs.cereg_drop_ms and (now_ms - cs.cereg_drop_ms > grace_ms) then
        gcs:send_text(MAV_SEVERITY.WARNING, 'LTE: network lost — fast reconnect')
        if not cs.disconnect_ms then cs.disconnect_ms = cs.cereg_drop_ms:tofloat() end
        cs.cereg_drop_ms = nil; cs.cipopen_sent = false; cs.hard_reset_strikes = 0
        cs.tx_state = "idle"; cs.tx_pending = ""
        if modem.cipclose then AT_send(modem.cipclose) end
        step = "CREG"; return
    end

    s = ser_device:readstring(512)
    if s then buf.modem = buf.modem .. s end
    if #buf.modem > 10240 then buf.modem = "" end
    if #buf.fc > 10240 then buf.fc = "" end

    -- Uplink (vehicle -> modem -> GCS)
    if cs.direct_push then
        local budget = 1024
        if P.TX_RATE:get() > 0 then
            budget = math.floor((now_ms - cs.last_send_data_ms):tofloat()*0.001 * P.TX_RATE:get())
        end
        dp.process_tx(now_ms, budget)
    else
        local quota = 0
        if P.TX_RATE:get() > 0 then quota = math.floor((now_ms - cs.last_send_data_ms):tofloat()*0.001 * P.TX_RATE:get()) end
        local data_sent = 0
        while #buf.modem > 0 do
            local n = #buf.modem
            if n > 100 then n = 100 end
            if quota > 0 and quota - data_sent < n then n = quota - data_sent end
            local data = buf.modem:sub(1, n)
            data_sent = data_sent + #data; cs.last_send_data_ms = now_ms
            if not data_send_connected(data) then break end
            buf.modem = buf.modem:sub(n + 1)
            if quota > 0 and data_sent >= quota then break end
        end
    end

    -- Deliver downlink payload to flight controller
    if #buf.fc > 0 then
        local nwritten = ser_device:writestring(buf.fc)
        if nwritten > 0 then buf.fc = buf.fc:sub(nwritten + 1) end
    end

    -- Signal acquisition
    if not option_enabled(OPT.NOSIGQUERY) then
        if cs.direct_push then
            -- RSRP/RSRQ/RSSI arrive via +QCSQ URC. Poll QENG slowly for band/CID,
            -- but only when the QISEND handshake is idle (never interleave a send).
            if cs.tx_state == "idle" and modem.preflight and now_ms - cs.last_CSQ_ms > 5000 then
                cs.last_CSQ_ms = now_ms; AT_send(modem.preflight)
            end
        elseif cmux_enabled() then
            if now_ms - cs.last_CSQ_ms > 500 then
                cs.last_CSQ_ms = now_ms
                if not modem.cpsi then AT_send("AT+CSQ\r\n")
                else
                    if cs.csq_toggle then AT_send("AT+CSQ\r\n") else AT_send(modem.cpsi) end
                    cs.csq_toggle = not cs.csq_toggle
                end
            end
        else
            if now_ms - cs.last_CSQ_ms > 1500 then
                cs.last_CSQ_ms = now_ms; AT_send(modem.cpsi or "AT+CSQ\r\n")
            end
        end

        -- RSSI heartbeat: skip in direct push (QCSQ supplies real RSSI).
        if not cs.direct_push and now_ms - cs.last_CSQ_reply_ms > 5000 then
            cs.last_CSQ_reply_ms = now_ms; gcs:send_named_float('LTE_RSSI', -1)
        end
        if P.MCCMNC:get() ~= last_mccmnc and modem.mccmnc then
            set_MCCMNC(); gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE_modem: set MCCMNC=%d", last_mccmnc)); step = "CREG"
        end
        if P.BAND:get() ~= last_band and (modem.setband or modem.setband_mask) then
            set_BAND(); if last_band ~= 0 then step = "CREG" end
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE_modem: set BAND=%d", last_band))
        end
    end

    if supports_routing and now_ms - cs.last_route_ms > 1000 then
        cs.last_route_ms = now_ms
        local dest = uint32_t(P_ROUTE.IP0:get())<<24 | uint32_t(P_ROUTE.IP1:get())<<16 | uint32_t(P_ROUTE.IP2:get())<<8 | uint32_t(P_ROUTE.IP3:get())
        if dest ~= uint32_t(0) then networking:add_route(0, 1, dest, math.floor(P_ROUTE.MASK:get())) end
    end
    if cs.direct_push and (not cs.last_tx_dbg_ms or now_ms - cs.last_tx_dbg_ms > 1000) then
        cs.last_tx_dbg_ms = now_ms
        local st_code = (cs.tx_state == "idle" and 0) or (cs.tx_state == "prompt" and 1) or 2  -- 2=sendok
        logger:write("LTED", 'QIS,PRM,OK,STL,RCV,TXQ,ST', 'IIIIIIB',
                     cs.dbg_qisend, cs.dbg_prompt, cs.dbg_sendok, cs.dbg_stall,
                     cs.dbg_recv, #buf.modem, st_code)
    end
end

local function run_step()
    if cs.change_baud then uart:begin(cs.change_baud); cs.change_baud = nil end
    local step_changed = (cs.last_step ~= step)
    local now_ms = millis()

    if step_changed then
        gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE_modem: step %s', step))
        if cs.last_step and cs.last_step ~= "ATI" then table.insert(cs.step_times, {name=cs.last_step, ms=math.floor(now_ms:tofloat()-cs.step_timer_ms)}) end
        cs.step_timer_ms = now_ms:tofloat()
        
        -- Diagnostic Timing Dump
        if step == "CONNECTED" and #cs.step_times > 0 then
            cs.post_reset = false -- Clear post-reset flag since connection succeeded
            local parts, total = {}, 0
            local max_name, max_ms = "none", 0
            for _, t in ipairs(cs.step_times) do 
                table.insert(parts, t.name..':'..t.ms..'ms')
                total = total + t.ms 
                if t.ms > max_ms then
                    max_ms = t.ms
                    max_name = t.name
                end
            end
            
            gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE longest step: %s (%dms)', max_name, max_ms))
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE timing: '..table.concat(parts,' ')..' total:'..total..'ms')
            
            if cs.disconnect_ms then
                local outage_s = (now_ms:tofloat() - cs.disconnect_ms) / 1000
                gcs:send_text(MAV_SEVERITY.WARNING,
                    string.format('LTE: total outage %.1fs', outage_s))
                cs.disconnect_ms = nil
            end
            cs.step_times = {}
        end
    end
    cs.last_step = step

    if step == "CONNECTED" then step_CONNECTED(); return 5 end

    local time_in_step = (now_ms:tofloat() - cs.step_timer_ms) / 1000
    if not step_changed and step ~= "ATI" and step ~= "CMUX" and step ~= "CPIN" and step ~= "CREG" and step ~= "HALT" then
        if time_in_step > P.STUCK_T:get() then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("LTE: %s timeout after %ds", step, P.STUCK_T:get()))
            reset_to_ATI(); return 1000
        end
    end

    if not step_changed and (step == "CPIN" or step == "CREG") then
        local creg_patience = math.max(30, P.STUCK_T:get() * 3)
        if time_in_step > creg_patience then
            gcs:send_text(MAV_SEVERITY.CRITICAL, "LTE: " .. step .. " search failed, hard reset")
            reset_to_ATI(); return 1000
        end
    end

    if step == "HALT" then
        local time_in_halt = (now_ms:tofloat() - cs.step_timer_ms) / 1000
        if math.floor(time_in_halt) % 15 == 0 then
            gcs:send_text(MAV_SEVERITY.CRITICAL, "LTE HALTED: SIM unresponsive. Power cycle modem.")
        end
        return 5000
    end
    if step == "ATI" then step_ATI(); return 1100 end
    if step == "BAUD" then step_BAUD(); return 50 end
    if step == "CREG" then step_CREG(); return 150 end 
    if step == "SIGNAL_GATE" then step_SIGNAL_GATE(); return 50 end
    if step == "SOCKET_STATE" then step_SOCKET_STATE(); return 50 end
    if step == "CGACT" then step_CGACT(); return 500 end
    if step == "CIPMODE" then step_CIPMODE(); return 200 end
    if step == "NETOPEN" then step_NETOPEN(); return 200 end
    if step == "CONFIG" then step_CONFIG(); return 50 end
    if step == "CMUX" then step_CMUX(); return 50 end
    if step == "CPIN" then step_CPIN(); return 50 end
    if step == "PPPOPEN" then step_PPPOPEN(); return 200 end
    if step == "CIPCLOSE" then step_CIPCLOSE(); return 50 end
    if step == "QENG" then step_QENG(); return 50 end
    if step == "CIPOPEN" then step_CIPOPEN(); return 50 end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("LTE_modem: bad step %s", step))
    reset_to_ATI()
end

local function update()
    if P.ENABLE:get() == 0 then return 500 end
    local delay = run_step()
    uart_write_pending()
    return delay
end

gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: starting')

function protected_wrapper()
   local ok, res = pcall(update)
   if not ok then
      gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem error: ' .. tostring(res))
      reset_state()
      return protected_wrapper, 1000
   end
   return protected_wrapper, res
end

return protected_wrapper, 500