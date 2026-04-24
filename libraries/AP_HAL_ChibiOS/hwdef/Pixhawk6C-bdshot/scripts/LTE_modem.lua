--[[
    driver for LTE modems with AT command set
    supported chipsets:
    - SIM7600
    - EC200
    - EC20
    - EC25
    - Air780
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "LTE_"

local PPP = 48

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'LTE_modem: could not add param table')

local LTE_ENABLE      = bind_add_param('ENABLE',  1, 1)
local LTE_SERPORT     = bind_add_param('SERPORT',  2, 0)
local LTE_SCRPORT     = bind_add_param('SCRPORT',  3, 0)
local LTE_SERVER_IP0  = bind_add_param('SERVER_IP0',  4, 0)
local LTE_SERVER_IP1  = bind_add_param('SERVER_IP1',  5, 0)
local LTE_SERVER_IP2  = bind_add_param('SERVER_IP2',  6, 0)
local LTE_SERVER_IP3  = bind_add_param('SERVER_IP3',  7, 0)
local LTE_SERVER_PORT = bind_add_param('SERVER_PORT',  8, 0)
local LTE_BAUD        = bind_add_param('BAUD',  9, 115200)
local LTE_TIMEOUT     = bind_add_param('TIMEOUT', 10, 10)
local LTE_PROTOCOL    = bind_add_param('PROTOCOL', 11, 48)
local LTE_OPTIONS     = bind_add_param('OPTIONS', 12, 0)
local LTE_IBAUD       = bind_add_param('IBAUD', 13, 115200)
local LTE_MCCMNC      = bind_add_param('MCCMNC', 14, -1)

local supports_routing = networking and networking.add_route -- luacheck: ignore 143

if supports_routing then
    LTE_ROUTE_IP0  = bind_add_param('ROUTE_IP0',  15, 0)
    LTE_ROUTE_IP1  = bind_add_param('ROUTE_IP1',  16, 0)
    LTE_ROUTE_IP2  = bind_add_param('ROUTE_IP2',  17, 0)
    LTE_ROUTE_IP3  = bind_add_param('ROUTE_IP3',  18, 0)
    LTE_ROUTE_MASK = bind_add_param('ROUTE_MASK',  19, 32)
end

local LTE_TX_RATE  = bind_add_param('TX_RATE',  20, 0)
local LTE_BAND      = bind_add_param('BAND', 21, -1)

LTE_OPTIONS_LOGALL  = (1<<0)
LTE_OPTIONS_SIGNALS = (1<<1)
LTE_OPTIONS_NOMUX   = (1<<2)
LTE_OPTIONS_NOSIGQUERY = (1<<3)
LTE_OPTIONS_TCP = (1<<4)

--[[
    AT command mappings (Consolidated into one table to save local variable slots)
--]]
local modem_list = {
    ["SimCom"] = { banner = 'SIMCOM', cmux = 'AT+CMUX=0\r\n', setbaud = 'AT+IPR=%u\r\n', pppopen = 'ATD*99#\r', cpin = 'AT+CPIN?\r\n', cpsi = 'AT+CPSI?\r\n', reset = 'AT+CFUN=1,1\r\n', cipmode = 'AT+CIPMODE=1\r\n', cipopen_udp = 'AT+CIPOPEN=0,"UDP","%d.%d.%d.%d",%d,6001\r\n', cipopen_tcp = 'AT+CIPOPEN=0,"TCP","%d.%d.%d.%d",%d\r\n', cgerep = 'AT+CGEREP=1,1\r\n', netopen = 'AT+NETOPEN\r\n', mccmnc = 'AT+COPS=1,2,"%u"\r\n', setband_mask = 'AT+CNBP=,0x%x\r\n', setband_all = 'AT+CNBP=,0x480000000000000000000000000000000000000000000042000007FFFFDF3FFF\r\n', config_extra = 'ATH\r\n' },
    ["SimCom2"] = { banner = 'R1951', cmux = 'AT+CMUX=0\r\n', setbaud = 'AT+IPR=%u\r\n', pppopen = 'ATD*99#\r', cpin = 'AT+CPIN?\r\n', cpsi = 'AT+CPSI?\r\n', cipmode = 'AT+CACID=0\r\n', cipopen_tcp = 'AT+CAOPEN=0,0,"TCP","%d.%d.%d.%d",%d\r\n', cipopen_udp = 'AT+CAOPEN=0,0,"UDP","%d.%d.%d.%d",%d\r\n', cgact = 'AT+CGACT?\r\n', cgerep = 'AT+CGEREP=1,1\r\n', reset = 'AT+CFUN=1,1\r\n', netopen = "AT+CNACT=0,1\r\n", netclose = "AT+CNACT=0,0\r\n", cfun = 'AT+CFUN=1\r\n', reset_not_baudrate = true, mccmnc = 'AT+COPS=4,2,"%u"\r\n', caswitch = 'AT+CASWITCH=0,1\r\n', setband = 'AT+CBANDCFG="CAT-M",%d\r\n', setband_all = 'AT+CBANDCFG="CAT-M",1,2,3,4,5,8,12,13,14,18,19,20,25,26,27,28,66,85\r\n' },
    ["Air780"] = { banner = 'AirM2M_780E', cmux = nil, setbaud = 'AT+IPR=%u\r\n', cgact = 'AT+CGACT=1,1\r\n', pppopen = 'ATD*99#\r', cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipmode = 'AT+CIPMODE=1\r\n' },
    ["EC200"] = { banner = 'EC200', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n' },
    ["EC20"] = { banner = 'EC20', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n' },
    ["EC25"] = { banner = 'EC25', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n' },
    ["BG95"] = { banner = 'BG95', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n' },
    ["EG800Q"] = { banner = 'EG800Q', cmux = 'AT+CMUX=0\r\n', pppopen = 'ATD*99#\r', cpsi = 'AT+QENG="servingcell"\r\n', cipmode = nil, cpin = 'AT+CPIN?\r\n', reset = 'AT+CFUN=1,1\r\n', cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n', cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n', cipclose = 'AT+QICLOSE=0\r\n', mccmnc = 'AT+COPS=4,2,"%u"\r\n', setband_mask = 'AT+QCFG="band",0,0x%x\r\n', setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n' }
}

local default_modem = { reset = 'AT+CFUN=1,1\r\r' }
local modem = default_modem

local function option_enabled(option)
    return (LTE_OPTIONS:get() & option) ~= 0
end

if LTE_ENABLE:get() == 0 then return end

local uart = serial:find_serial(LTE_SERPORT:get())
if not uart then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find serial port')
    return
end

local ser_device = serial:find_simulated_device(LTE_PROTOCOL:get(), LTE_SCRPORT:get())
if not ser_device then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find SCR_SDEV device')
    return
end

local step = "ATI"
local stats = { bytes_in = 0, bytes_out = 0 }

-- State machine timing variables
local step_times = {}
local step_timer_ms = millis():tofloat()

uart:begin(LTE_IBAUD:get())

local log_file = io.open('LTE_modem.log', 'w')
local function log_data(s, marker)
    if s and #s > 0 and log_file then
        log_file:write(marker .. '[' .. s .. ']\n')
        log_file:flush()
    end
end

local function uart_read()
    local s = uart:readstring(512)
    if not s then return "" end
    log_data(s, '<<<')
    stats.bytes_in = stats.bytes_in + #s
    return s
end

local pending_to_uart = ""

local function uart_write_pending()
    if #pending_to_uart > 0 then
        local n = uart:writestring(pending_to_uart)
        pending_to_uart = pending_to_uart:sub(n+1)
    end
end

local function uart_write(s)
    pending_to_uart = pending_to_uart .. s
    if option_enabled(LTE_OPTIONS_LOGALL) or step ~= "CONNECTED" then
        log_data(s, '>>>')
    end
    stats.bytes_out = stats.bytes_out + #s
    return #s
end

-- Constants for GSM 07.10 CMUX framing
local FLAG = 0xF9
local UIH = 0xEF
local SABM = 0x2F
local EA = 0x01
local CR_SEND = 0x02
local DLC_AT = 1
local DLC_DATA = 2

-- CMUX buffer state
local cmux = {}
cmux.buffers = {[DLC_AT] = "", [DLC_DATA] = ""} 

local last_mccmnc = nil
local last_band = nil

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
    local addr = string.char((dlc << 2) | EA | CR_SEND)
    local ctrl = string.char(dtype | 0x10)
    local len = #data
    local len_byte = string.char((len << 1) | EA)
    local header = addr .. ctrl .. len_byte
    local fcs = string.char(fcs_calc(header))
    return string.char(FLAG) .. header .. data .. fcs .. string.char(FLAG)
end

local found_cmux = false

local function cmux_enabled()
    if found_cmux then return true end
    return modem and modem.cmux and not option_enabled(LTE_OPTIONS_NOMUX)
end

local function AT_send(atcmd)
    local s = cmux_enabled() and cmux.encode_cmux_frame(DLC_AT, UIH, atcmd) or atcmd
    return uart_write(s) == #s
end

local function send_data_reset()
    if modem.reset then
        AT_send(modem.reset)
        if not modem.reset_not_baudrate then
            uart:begin(LTE_IBAUD:get())
        end
        found_cmux = false
        gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: sent reset")
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
    uart_write(cmux.encode_cmux_frame(0, SABM, ""))
    uart_write(cmux.encode_cmux_frame(1, SABM, ""))
    uart_write(cmux.encode_cmux_frame(2, SABM, ""))
end

function cmux.parse_cmux_frame(buf)
    local start_idx = buf:find(string.char(FLAG))
    if not start_idx then return nil, nil, nil end
    if #buf < 6 then return nil, nil, nil, "short" end
    local len_byte = buf:byte(4)
    if (len_byte & EA) == 0 then return nil, nil, nil end
    local len = len_byte >> 1
    local end_idx = 6 + len
    if buf:byte(end_idx) ~= FLAG then return nil, nil, nil, "short" end

    local frame = buf:sub(start_idx + 1, end_idx - 1)
    if #frame < 4 then return nil, nil, nil end

    local addr = frame:byte(1)
    local ctrl = frame:byte(2)

    if ctrl == SABM then return nil, nil, buf:sub(end_idx + 1) end
    if (ctrl & 0xef) ~= UIH then return nil, nil, nil end
    if #frame ~= 3 + len + 1 then return nil, nil, nil end

    local data = frame:sub(4, 3 + len)
    local fcs_field = frame:byte(3 + len + 1)
    local header = frame:sub(1, 3)
    if fcs_calc(header) ~= fcs_field then return nil, nil, nil end

    local dlc = (addr >> 2) & 0x3F
    local remainder = buf:sub(end_idx + 1)
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
    local s = cmux_enabled() and cmux.encode_cmux_frame(DLC_DATA, UIH, data) or data
    return uart_write(s) == #s
end

local function data_send_connected(data)
    local s = cmux_enabled() and cmux.encode_cmux_frame(DLC_DATA, UIH, data) or data
    local n = uart_write(s)
    stats.bytes_out = stats.bytes_out + n
    return n == #s
end

local ati_sequence = 0
local last_data_ms = millis()
local pending_to_modem = ""
local pending_to_fc = ""
local pending_to_parse = ""

local function reset_buffers()
    last_data_ms = millis()
    pending_to_modem = ""; pending_to_fc = ""; pending_to_parse = ""
    cmux.buffers[DLC_AT] = ""; cmux.buffers[DLC_DATA] = ""
    while ser_device:available() > 0 do ser_device:readstring(512) end
end

local function reset_state()
    step = "ATI"; modem = default_modem; found_cmux = false
    reset_buffers(); pending_to_uart = ""
    step_times = {}; step_timer_ms = millis():tofloat()
end

local function reset_to_ATI()
    send_data_reset(); uart_write_pending(); reset_state()
end

local function check_modem_banner(s)
    for model in pairs(modem_list) do
        if s:find(modem_list[model].banner) then
            modem = modem_list[model]
            gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: found modem: " .. model)
            return
        end
    end
end

local function step_ATI()
    local s = uart_read()
    if s and modem == default_modem then check_modem_banner(s) end
    if modem ~= default_modem then
        if not cmux_enabled() then step = "BAUD" else step = "CMUX" end
        return
    end
    if not option_enabled(LTE_OPTIONS_NOMUX) and s and #s >= 4 and s:byte(1) == FLAG and s:byte(-1) == FLAG then
        found_cmux = true
        gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: in CMUX mode")
        log_data("{INCMUX}", '***')
        AT_send('ATI\r')
        return
    end
    if ati_sequence % 3 == 2 then
        uart_write('+++')
    elseif ati_sequence % 3 == 1 and not option_enabled(LTE_OPTIONS_NOMUX) then
        uart_write(cmux.encode_cmux_frame(DLC_AT, UIH, "ATI\r"))
    else
        uart_write('\rATI\r')
    end
    if ati_sequence % 10 == 5 then
        uart:begin(LTE_BAUD:get()); log_data(string.format("{BAUD=%d}", LTE_BAUD:get()), '***')
    end
    if ati_sequence % 10 == 9 then
        uart:begin(LTE_IBAUD:get()); log_data(string.format("{BAUD=%d}", LTE_IBAUD:get()), '***')
    end
    ati_sequence = ati_sequence + 1
end

local change_baud = nil

local function step_BAUD()
    if modem.setbaud and LTE_BAUD:get() ~= LTE_IBAUD:get() then
        change_baud = LTE_BAUD:get()
        AT_send(string.format(modem.setbaud, change_baud))
    end
    step = "CPIN"
end

local function set_MCCMNC()
    if not modem.mccmnc then return end
    local mccmnc = math.floor(LTE_MCCMNC:get())
    if mccmnc > 0 then AT_send(string.format(modem.mccmnc, mccmnc))
    elseif mccmnc == 0 then AT_send("AT+COPS=0\r\n") end
    last_mccmnc = mccmnc
end

local function set_BAND()
    if not modem.setband and not modem.setband_mask then return end
    local band = math.floor(LTE_BAND:get())
    if band > 0 then
       if modem.setband_mask then AT_send(string.format(modem.setband_mask, 1<<(band-1)))
       else AT_send(string.format(modem.setband, band)) end
    elseif band == 0 then AT_send(modem.setband_all) end
    last_band = band
end

local function step_CONFIG()
    set_BAND(); set_MCCMNC()
    if modem.config_extra then AT_send(modem.config_extra) end
    step = "CREG"
end

local function step_CPIN()
    local s = uart_read()
    if s and s:find("READY") then step = "CONFIG" end
    AT_send('AT+CPIN?\r\n')
end

local function step_CREG()
    local s = uart_read()
    if handle_error(s) then return end
    if s then
        if cmux_enabled() and #s > 4 and not cmux.parse_cmux_frame(s) then
            step = "CMUX"; return
        end
        local reg = s:match('CREG: %d,(%d+)\r\n')
        if reg == "1" or reg == "5" then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CREG OK')
            if LTE_PROTOCOL:get() == PPP then
                if modem.cgact then step = "CGACT" else step = "PPPOPEN" end
            else
                if modem.cipmode then step = "CIPMODE" else step = "CIPOPEN" end
            end
            return
        elseif reg then
            if reg == "0" then AT_send("AT+CFUN=1\r\n"); AT_send("AT+COPS?\r\n") end
        end
    end
    AT_send('AT+CREG?\r\n')
end

local function step_CGACT()
    local s = uart_read()
    if handle_error(s) then return end
    if s and s:find('\r\nOK\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CGACT OK')
        if LTE_PROTOCOL:get() == PPP then step = "PPPOPEN"; last_data_ms = millis()
        else step = "CIPMODE" end
        return
    end
    data_send(modem.cgact)
    if modem.cfun then data_send(modem.cfun) end
end

local function step_CIPMODE()
    local s = uart_read()
    if s:find('AT+CACID=0,0') then gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network context set'); step = "NETOPEN"; return end
    if handle_error(s) then return end
    if s:find('\r\r\nOK\r') then gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: transparent mode set'); step = "NETOPEN"; return end
    data_send(modem.cipmode)
end

local function step_CMUX()
    local s = uart_read()
    if s then
        if s:find("CME ERROR") then AT_send('AT+CFUN=1\r\n')
        elseif #s >= 4 and (cmux.parse_cmux_frame(s) or s:find('CMUX=0\r\r\nOK\r') or s == string.char(FLAG,FLAG,FLAG,FLAG)) then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CMUX mode set')
            cmux.send_sabm(); step = "BAUD"; return
        end
    end
    uart_write(modem.cmux)
end

local function step_NETOPEN()
    if not modem.netopen then step = "CIPOPEN"; return end
    local s = uart_read()
    if s:find("AT+CNACT=0,1") and s:find("ERROR") and modem.netclose then data_send(modem.netclose); return end
    if handle_error(s) then return end
    if s and (s:find('NETOPEN\r') or s:find('ACTIVE\r')) and s:find('OK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network opened'); step = "CIPOPEN"; return
    end
    data_send(modem.netopen)
end

local function step_PPPOPEN()
    local s = uart_read()
    if s and modem.cgact and s:find("\r\nNO CARRIER\r\n") then send_data_reset(); step = "ATI"; return end
    if s and s:find("CME ERROR:") then send_data_reset(); step = "ATI"; return end
    if handle_error(s) then return end
    if s and s:find('CONNECT') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected'); reset_buffers(); step = "CONNECTED"; return
    end
    data_send(modem.pppopen)
end

local function step_CIPOPEN()
    local s = uart_read()
    if handle_error(s) then return end
    if s then
        if s == "" and modem.cipclose then AT_send(modem.cipclose) end
        if s:find('+CAOPEN: 0,0') and s:find('OK\r') and modem.caswitch then data_send(modem.caswitch); return end
        if s:find('CONNECT') or (s:find('+CAOPEN: 0,0') and s:find('OK\r')) then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
            reset_buffers(); step = "CONNECTED"; return
        end
    end
    if LTE_SERVER_PORT:get() <= 0 then gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT"); return end
    local cipopen = option_enabled(LTE_OPTIONS_TCP) and modem.cipopen_tcp or modem.cipopen_udp
    data_send(string.format(cipopen, LTE_SERVER_IP0:get(), LTE_SERVER_IP1:get(), LTE_SERVER_IP2:get(), LTE_SERVER_IP3:get(), LTE_SERVER_PORT:get()))
end

local function check_CSQ(s)
    local rssi_raw, ber_raw = s:match("%+CSQ:%s*(%d+),(%d+)")
    if rssi_raw then
        gcs:send_named_float('LTE_RSSI', rssi_raw)
        logger:write("LTE",'RSSI,BER,Bin,Bout','iiII', rssi_raw, ber_raw, stats.bytes_in, stats.bytes_out)
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
    local system_mode, operation_mode, mcc_mnc, tac_str, scell_id_str, pcid_str, earfcn_band, ul_freq_str, dl_freq_str, tdd_cfg_str, rsrq_str, rsrp_str, rssi_str, sinr_str = s:match("+CPSI:%s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([%-]?%d+),([%-]?%d+),([%-]?%d+),([%-]?%d+)")
    if system_mode and sinr_str then
        local tac = tonumber(tac_str:match("0x(%w+)"), 16) or tonumber(tac_str) or 0
        local scell_id = tonumber(scell_id_str) or 0
        local band = earfcn_band:match("[^%d]+(%d+)") or -1
        logger:write("LTES",'Md,Op,MCC,TAC,CID,PID,BND,F,DF,TDD,RP,RQ,RS,SR','NNNIIINHhhhhhh', system_mode, operation_mode, mcc_mnc, tac, scell_id, tonumber(pcid_str) or 0, earfcn_band, tonumber(ul_freq_str) or 0, tonumber(dl_freq_str) or 0, tonumber(tdd_cfg_str) or 0, tonumber(rsrp_str) or 0, tonumber(rsrq_str) or 0, tonumber(rssi_str) or 0, tonumber(sinr_str) or 0)
        return true
    end
    return false
end

local function check_QENG(s)
    if not s:find("+QENG") then return false end
    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))
    local mcc_str, mnc_str, cid_hex, pcid_str, earfcn_str, band_str, tac_hex, rsrp_str, rsrq_str, rssi_str, sinr_str = s:match('+QENG:%s+"servingcell","[^"]+","[^"]+","[^"]+",(%d+),(%d+),([%x]+),(%d+),(%d+),(%d+),%d+,%d+,([%x]+),([%-]?%d+),([%-]?%d+),([%-]?%d+)')
    if mcc_str then
        local tac = tonumber(tac_hex, 16) or 0
        local cid = tonumber(cid_hex, 16) or 0
        logger:write("LTES", 'MCC,MNC,TAC,CID,PID,EF,RSRP,RSRQ,RSSI,SINR', 'iiiiiiiiii', tonumber(mcc_str) or 0, tonumber(mnc_str) or 0, tac, cid, tonumber(pcid_str) or 0, tonumber(earfcn_str) or 0, tonumber(rsrp_str) or 0, tonumber(rsrq_str) or 0, tonumber(rssi_str) or 0, tonumber(sinr_str) or 0)
        return true
    end
    return false
end

local function handle_AT_reply(s)
    check_CSQ(s)
    if check_CPSI(s) or check_QENG(s) or check_CGACT(s) then return end
    if s:find("PPPD: DISCONNECTED") then step = "PPPOPEN" end
end

local last_CSQ_ms, last_CSQ_reply_ms, last_parse_ms, last_route_ms, last_send_data_ms = millis(), uint32_t(0), uint32_t(0), uint32_t(0), uint32_t(0)

local function step_CONNECTED()
    local s = uart:readstring(512)
    stats.bytes_in = stats.bytes_in + #s
    if option_enabled(LTE_OPTIONS_LOGALL) then log_data(s, '<<<') end
    if s and s:find('\r\nCLOSED\r\n') then gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connection closed, reconnecting'); step = "CIPOPEN"; return end
    if s and s:find('PPPD: DISCONNECTED\r\n') then gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: PPP closed, reconnecting'); step = "PPPOPEN"; return end

    local now_ms = millis()
    if s and #s > 0 then
        if not cmux_enabled() then
            pending_to_fc = pending_to_fc .. s; last_data_ms = now_ms
        else
            pending_to_parse = pending_to_parse .. s
            pending_to_parse = cmux.feed_uart_in(pending_to_parse)
            if now_ms - last_parse_ms > 1000 then pending_to_parse = "" end
            if #cmux.buffers[DLC_AT] > 0 then
                last_parse_ms = now_ms; handle_AT_reply(cmux.buffers[DLC_AT]); cmux.buffers[DLC_AT] = ""
            end
            if #cmux.buffers[DLC_DATA] > 0 then
                last_data_ms = now_ms; last_parse_ms = now_ms; pending_to_fc = pending_to_fc .. cmux.buffers[DLC_DATA]; cmux.buffers[DLC_DATA] = ""
            end
        end
    elseif LTE_TIMEOUT:get() > 0 and now_ms - last_data_ms > uint32_t(LTE_TIMEOUT:get() * 1000) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: timeout'); reset_to_ATI(); return
    end
    
    s = ser_device:readstring(512)
    if s then pending_to_modem = pending_to_modem .. s end

    if #pending_to_modem > 10240 then pending_to_modem = "" end
    if #pending_to_fc > 10240 then pending_to_fc = "" end

    local quota = 0
    if LTE_TX_RATE:get() > 0 then quota = math.floor((now_ms - last_send_data_ms):tofloat()*0.001 * LTE_TX_RATE:get()) end

    local data_sent = 0
    while #pending_to_modem > 0 do
        local n = #pending_to_modem
        if n > 100 then n = 100 end
        if quota > 0 and quota - data_sent < n then n = quota - data_sent end
        local data = pending_to_modem:sub(1, n)
        data_sent = data_sent + #data; last_send_data_ms = now_ms
        if not data_send_connected(data) then break end
        pending_to_modem = pending_to_modem:sub(n + 1)
        if quota > 0 and data_sent >= quota then break end
    end
    
    if #pending_to_fc > 0 then
        local nwritten = ser_device:writestring(pending_to_fc)
        if nwritten > 0 then pending_to_fc = pending_to_fc:sub(nwritten + 1) end
    end

    if cmux_enabled() and not option_enabled(LTE_OPTIONS_NOSIGQUERY) then
        if now_ms - last_CSQ_ms > 1000 then
            last_CSQ_ms = now_ms; AT_send("AT+CSQ\r\n")
            if modem.cpsi then AT_send(modem.cpsi) end
        end
        if now_ms - last_CSQ_reply_ms > 5000 then last_CSQ_reply_ms = now_ms; gcs:send_named_float('LTE_RSSI', -1) end
    end
end

local step_count = 0
local last_step = nil

local function run_step()
    if change_baud then uart:begin(change_baud); change_baud = nil end

    -- Check if we transitioned to a new state
    local step_changed = (last_step ~= step)

    -- TIMING REPORT LOGIC
    if step_changed then
        if last_step and last_step ~= "ATI" then 
            table.insert(step_times, {name=last_step, ms=math.floor(millis():tofloat() - step_timer_ms)}) 
        end
        step_timer_ms = millis():tofloat()
        
        if step == "CONNECTED" and #step_times > 0 then
            local parts, total = {}, 0
            for _, t in ipairs(step_times) do table.insert(parts, t.name..':'..t.ms..'ms'); total = total + t.ms end
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE timing: '..table.concat(parts,' ')..' total:'..total..'ms')
            step_times = {} 
        end
    end

    -- Update last_step BEFORE the early return
    last_step = step

    -- EARLY RETURN FOR DATA STREAM (200Hz)
    if step == "CONNECTED" then 
        step_CONNECTED()
        step_count = 0 
        return 5 
    end

    -- STUCK GUARD: Prevent infinite loops in a broken state
    if not step_changed and step ~= "ATI" and step ~= "CPIN" then
        step_count = step_count + 1
        if step_count > 50 then 
            gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: step reset")
            reset_to_ATI() 
            return 1000 -- Back off for 1 second after a reset
        end
    else
        step_count = 0 -- Reset counter when moving to a new step
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE_modem: step %s', step))

    -- STATE HANDLERS
    if step == "ATI" then step_ATI(); return 1100 end
    if step == "BAUD" then step_BAUD(); return 500 end
    if step == "CREG" then step_CREG(); return 1000 end
    if step == "CGACT" then step_CGACT(); return 500 end
    if step == "CIPMODE" then step_CIPMODE(); return 200 end
    if step == "NETOPEN" then step_NETOPEN(); return 200 end
    if step == "CONFIG" then step_CONFIG(); return 200 end
    if step == "CMUX" then step_CMUX(); return 200 end
    if step == "CPIN" then step_CPIN(); return 500 end
    if step == "PPPOPEN" then step_PPPOPEN(); return 200 end
    if step == "CIPOPEN" then step_CIPOPEN(); return 200 end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("LTE_modem: bad step %s", step))
    reset_to_ATI()
end

local function update()
    if LTE_ENABLE:get() == 0 then return 500 end
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