--[[
    driver for LTE modems with AT command set
    supported chipsets:
    - SIM7600
    - EC200
    - EC20
    - EC25
    - Air780

    Changes from original:
    1. CREG → CEREG — AT+CEREG? is now sent for EC25/EC20/EC200/BG95/EG800Q. 
    Parses +CEREG: n,stat as the primary match, with +CREG as fallback for SIM7600 and other non-LTE modems.
    2. COPS re-scan on not-registered — if CEREG returns stat 0 (not searching) or 3 (denied), 
    the script sends AT+COPS=2 (deregister) then AT+COPS=0 (auto-select), loops back to CEREG polling.
    3. QCSQ signal gate — new step_QCSQ_GATE runs after CREG OK. Sends AT+QCSQ, parses RSRP. 
    If ≥ -100 dBm → proceeds. If weak → sends AT+COPS=2 + AT+COPS=0 to force tower reselection, 
    goes back to CREG. After 3 failed re-scans → connects anyway with a warning.
    4. QISTATE before CIPOPEN — new step_QISTATE sends AT+QISTATE?. If +QISTATE: 0 is in the response → closes first. 
    If OK with no QISTATE → goes straight to CIPOPEN. Replaces the error-detect-then-close approach.
    5. CIPOPEN retry 3× — cipopen_sent_ms tracks when CIPOPEN was sent. After 3 seconds of no +QIOPEN URC → sends QICLOSE + retries. 
    After 3 retries → full reset. The stuck-guard (50 iterations) no longer handles this case.
    6. CEREG URC fast reconnect — in step_CONNECTED, if +CEREG: 0 or +CREG: 0,0 arrives as an unsolicited report → immediately closes socket 
    and jumps to CREG step. This catches network drops the moment they happen instead of waiting for the 10-30s timeout.
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "LTE_"

-- local MAVLINK2 = 2
local PPP = 48

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'LTE_modem: could not add param table')

--[[
    // @Param: LTE_ENABLE
    // @DisplayName: LTE Enable
    // @Description: Enable or disable the LTE modem driver
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local LTE_ENABLE = bind_add_param('ENABLE',  1, 1)

--[[
    // @Param: LTE_SERPORT
    // @DisplayName: Serial Port
    // @Description: Serial port to use for the LTE modem. This is the index of the SERIALn_ ports that are set to 28 for "scripting"
    // @Range: 0 8
    // @User: Standard
--]]
local LTE_SERPORT = bind_add_param('SERPORT',  2, 0)

--[[
    // @Param: LTE_SCRPORT
    // @DisplayName: Scripting Serial Port
    // @Description: Scripting Serial port to use for the LTE modem. This is the index of the SCR_SDEV ports that are set to 2 for "MAVLink2"
    // @Range: 0 8
    // @User: Standard
--]]
local LTE_SCRPORT = bind_add_param('SCRPORT',  3, 0)

--[[
    // @Param: LTE_SERVER_IP0
    // @DisplayName: Server IP 0
    // @Description: First octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP0  = bind_add_param('SERVER_IP0',  4, 0)

--[[
    // @Param: LTE_SERVER_IP1
    // @DisplayName: Server IP 1
    // @Description: Second octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP1  = bind_add_param('SERVER_IP1',  5, 0)

--[[
    // @Param: LTE_SERVER_IP2
    // @DisplayName: Server IP 2
    // @Description: Third octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP2  = bind_add_param('SERVER_IP2',  6, 0)

--[[
    // @Param: LTE_SERVER_IP3
    // @DisplayName: Server IP 3
    // @Description: Fourth octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP3  = bind_add_param('SERVER_IP3',  7, 0)

--[[
    // @Param: LTE_SERVER_PORT
    // @DisplayName: Server Port
    // @Description: IPv4 Port of the server to connect to
    // @Range: 1 65525
    // @User: Standard
--]]
local LTE_SERVER_PORT = bind_add_param('SERVER_PORT',  8, 0)

--[[
    // @Param: LTE_BAUD
    // @DisplayName: Serial Baud Rate
    // @Description: Baud rate for the serial port to the LTE modem when connected. Initial power on baudrate is in LTE_IBAUD
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
    // @User: Standard
--]]
local LTE_BAUD        = bind_add_param('BAUD',  9, 115200)

--[[
    // @Param: LTE_TIMEOUT
    // @DisplayName: Timeout
    // @Description: Timeout in seconds for the LTE connection. If no data is received for this time, the connection will be reset. A value of zero disables the timeout
    // @Range: 0 60
    // @Units: s
    // @User: Standard
--]]
local LTE_TIMEOUT     = bind_add_param('TIMEOUT', 10, 10)

--[[
    // @Param: LTE_PROTOCOL
    // @DisplayName: LTE protocol
    // @Description: The protocol that we will use in communication with the LTE modem. If this is PPP then the LTE_SERVER parameters are not used and instead a PPP connection will be established and you should use the NET_ parameters to enable network ports. If this is MAVLink2 then the LTE_SERVER parameters are used to create a TCP or UDP connection to a single server.
    // @Values: 2:MavLink2,48:PPP
    // @User: Standard
--]]
local LTE_PROTOCOL     = bind_add_param('PROTOCOL', 11, 48)

--[[
    // @Param: LTE_OPTIONS
    // @DisplayName: LTE options
    // @Description: Options to control the LTE modem driver. If VerboseSignalInfoGCS is set then additional NAMED_VALUE_FLOAT values are sent with verbose signal information
    // @Bitmask: 0:LogAllData,1:VerboseSignalInfoGCS,2:DisableMultiplexing,3:DisableSignalQueries,4:UseTCP
    // @User: Standard
--]]
local LTE_OPTIONS     = bind_add_param('OPTIONS', 12, 0)

--[[
    // @Param: LTE_IBAUD
    // @DisplayName: LTE initial baudrate
    // @Description: This is the initial baud rate on power on for the modem. This is set in the modem with the AT+IREX=baud command
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
    // @User: Standard
--]]
local LTE_IBAUD       = bind_add_param('IBAUD', 13, 115200)

--[[
    // @Param: LTE_MCCMNC
    // @DisplayName: LTE operator selection
    // @Description: This allows selection of network operator
    // @Values: -1:NoChange,0:Default,50501:AU-Telstra,50502:AU-Optus,50503:AU-Vodafone
    // @User: Standard
--]]
local LTE_MCCMNC      = bind_add_param('MCCMNC', 14, -1)

local supports_routing = networking and networking.add_route -- luacheck: ignore 143

if supports_routing then
    -- add_route() API only on newer firmwares
    --[[
        // @Param: LTE_ROUTE_IP0
        // @DisplayName: custom route IP 0
        // @Description: First octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP0  = bind_add_param('ROUTE_IP0',  15, 0)

    --[[
        // @Param: LTE_ROUTE_IP1
        // @DisplayName: custom route IP 1
        // @Description: Second octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP1  = bind_add_param('ROUTE_IP1',  16, 0)

    --[[
        // @Param: LTE_ROUTE_IP2
        // @DisplayName: custom route IP 2
        // @Description: Third octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP2  = bind_add_param('ROUTE_IP2',  17, 0)

    --[[
        // @Param: LTE_ROUTE_IP3
        // @DisplayName: custom route IP 3
        // @Description: Fourth octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP3  = bind_add_param('ROUTE_IP3',  18, 0)

    --[[
        // @Param: LTE_ROUTE_MASK
        // @DisplayName: custom route netmask length
        // @Description: number of bits in route netmask. Use 32 for a single IP
        // @Range: 0 32
        // @User: Standard
    --]]
    LTE_ROUTE_MASK  = bind_add_param('ROUTE_MASK',  19, 32)
end

--[[
    // @Param: LTE_TX_RATE
    // @DisplayName: Max transmit rate
    // @Description: Maximum data transmit rate to the modem in bytes/second. Use zero for unlimited
    // @User: Standard
--]]
local LTE_TX_RATE  = bind_add_param('TX_RATE',  20, 0)

--[[
    // @Param: LTE_BAND
    // @DisplayName: LTE band selection
    // @Description: This allows selection of LTE band. A value of -1 means no band setting change is made. A value of 0 sets all bands. Otherwise the specified band is set.
    // @Range: -1 50
    // @User: Standard
--]]
local LTE_BAND      = bind_add_param('BAND', 21, -1)

local LTE_OPTIONS_LOGALL  = (1<<0)
local LTE_OPTIONS_SIGNALS = (1<<1)
local LTE_OPTIONS_NOMUX   = (1<<2)
local LTE_OPTIONS_NOSIGQUERY = (1<<3)
local LTE_OPTIONS_TCP = (1<<4)


--[[
    AT command mappings for different modem chipsets
--]]
local EC20  = { banner = 'EC20C',
                 cmux = 'AT+CMUX=0\r\n',
                 -- setbaud not used: sending AT+IPR after CMUX breaks framing
                 pppopen = 'ATD*99#\r',
                 cpsi = 'AT+QENG="servingcell"\r\n',
                 cipmode = nil,
                 cpin = 'AT+CPIN?\r\n',
                 reset = 'AT+CFUN=1,1\r\n',
                 cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n',
                 cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n',
                 cipclose = 'AT+QICLOSE=0\r\n',
                 mccmnc = 'AT+COPS=4,2,"%u"\r\n',
                 setband_mask = 'AT+QCFG="band",0,0x%x\r\n',
                 setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n',
                }
-- EC25 uses banner EC25C (matches EC25CEFILGR... but not EC200CNLGAR...).
-- AT command set is identical to EC20 — same Quectel platform and firmware family.
-- setbaud not used for same reason as EC20: AT+IPR after CMUX breaks framing.
local EC25  = { banner = 'EC25E',
                 cmux = 'AT+CMUX=0\r\n',
                 -- setbaud not used: sending AT+IPR after CMUX breaks framing
                 pppopen = 'ATD*99#\r',
                 cpsi = 'AT+QENG="servingcell"\r\n',
                 cipmode = nil,
                 cpin = 'AT+CPIN?\r\n',
                 reset = 'AT+CFUN=1,1\r\n',
                 cipopen_tcp = 'AT+QIOPEN=1,0,"TCP","%d.%d.%d.%d",%d,0,2\r\n',
                 cipopen_udp = 'AT+QIOPEN=1,0,"UDP","%d.%d.%d.%d",%d,6001,2\r\n',
                 cipclose = 'AT+QICLOSE=0\r\n',
                 mccmnc = 'AT+COPS=4,2,"%u"\r\n',
                 setband_mask = 'AT+QCFG="band",0,0x%x\r\n',
                 setband_all = 'AT+QCFG="band",0,0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\r\n',
                }
local default_modem = { reset = 'AT+CFUN=1,1\r\r' }

local modem_list = {
    ["EC20"] = EC20,
    ["EC25"] = EC25,
}

local modem = default_modem

--[[
    return true if an option is enabled
--]]
local function option_enabled(option)
    return (LTE_OPTIONS:get() & option) ~= 0
end

if LTE_ENABLE:get() == 0 then
    -- disabled
    return
end

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

uart:begin(LTE_IBAUD:get())

--[[
    Open a log file to log the output from the modem
    This is useful for debugging the connection process
--]]
local log_file = io.open('LTE_modem.log', 'w')

--[[
    log data to log_file
--]]
local function log_data(s, marker)
    if s and #s > 0 and log_file then
        log_file:write(marker .. '[' .. s .. ']\n')
        log_file:flush()
    end
end

--[[
    Function to read from the UART and log the output
    This function reads up to 512 bytes at a time and writes it to the log file
    returns the string read or nil
--]]
local function uart_read()
    local s = uart:readstring(512)
    if not s then
        return ""
    end
    log_data(s, '<<<')
    stats.bytes_in = stats.bytes_in + #s
    return s
end

local pending_to_uart = ""

--[[
    write any pending bytes to the uart
--]]
local function uart_write_pending()
    if #pending_to_uart > 0 then
        local n = uart:writestring(pending_to_uart)
        pending_to_uart = pending_to_uart:sub(n+1)
    end
end

--[[
    Function to write to the UART and log the command
--]]
local function uart_write(s)
    pending_to_uart = pending_to_uart .. s
    if option_enabled(LTE_OPTIONS_LOGALL) or step ~= "CONNECTED" then
        log_data(s, '>>>')
    end
    stats.bytes_out = stats.bytes_out + #s
    return #s
end

-- CMUX buffer state + GSM 07.10 framing constants (merged to stay under 100-local limit)
local cmux = {
    FLAG    = 0xF9,
    UIH     = 0xEF,
    SABM    = 0x2F,
    --UA    = 0x63,
    EA      = 0x01,
    CR_SEND = 0x02,
    DLC_AT   = 1,
    DLC_DATA = 2,
}
cmux.buffers = {[cmux.DLC_AT] = "", [cmux.DLC_DATA] = ""} -- DLC1=AT, DLC2=DATA(PPP or TCP)

local last_mccmnc = nil
local last_band = nil

-- tracking table for GCS band/cell-tower switch warnings (single local, avoids hitting the 100-local limit)
local lte_track = { band = nil, cid = nil }
local cell_cache = { mcc=0, mnc=0, tac=0, cid=0, pcid=0, earfcn=0, band=-1 }
local cs = {
    cops_rescanning = false, cops_rescan_t   = 0,
    qcsq_tries      = 0,
    cipopen_retry   = 0,    cipopen_sent    = false, cipopen_sent_ms = 0,
    step_count      = 0,    last_step       = nil,
    step_timer_ms   = 0,    step_times      = {},
}

--[[
    FCS lookup table for polynomial x^8 + x^2 + x^1 + 1 (0x07)
    This is the reverse of the standard CRC-8 table
--]]
local fcs_table = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
    0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
    0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
    0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
    0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
    0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
    0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
    0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
    0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
    0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
    0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
    0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
    0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
    0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
    0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
    0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
    0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf
}

--[[
    Calculate FCS for a byte array
    data: table of bytes (numbers 0-255) or string
    Returns: FCS value (0-255)
--]]
local function fcs_calc(data)
    local fcs = 0xff  -- Initial value
    
    for i = 1, #data do
        local byte = string.byte(data, i)
        fcs = fcs_table[((fcs ~ byte) & 0xff) + 1] ~ (fcs >> 8)
    end

    return (~fcs) & 0xff
end

-- Construct a CMUX frame for a given DLC, data type and data
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

--[[
    return true if we should use CMUX
--]]
local function cmux_enabled()
    if found_cmux then
        return true
    end
    return modem and modem.cmux and not option_enabled(LTE_OPTIONS_NOMUX)
end

--[[
    send an AT command string with possible CMUX framing
--]]
local function AT_send(atcmd)
    local s
    if cmux_enabled() then
        s = cmux.encode_cmux_frame(cmux.DLC_AT, cmux.UIH, atcmd)
    else
        s = atcmd
    end
    return uart_write(s) == #s
end

--[[
    send an appropriate data reset for the protocol
--]]
local function send_data_reset()
    if modem.reset then
        AT_send(modem.reset)
        if not modem.reset_not_baudrate then
            -- a reset changes the baud rate to the initial baud rate
            uart:begin(LTE_IBAUD:get())
        end
        -- and clears cmux state
        found_cmux = false
        gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: sent reset")
        return
    end
end

--[[
    Function to handle errors in the response from the modem
    If an error is detected, it resets the modem
    returns true if an error was detected
--]]
local function handle_error(s)
    if s and s:find('\nERROR\r\n') then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: error response from modem')
        send_data_reset()
        step = "ATI"
        return true
    end
    return false
end

-- Send SABM (Set Asynchronous Balanced Mode) for all DLCs
function cmux.send_sabm()
    uart_write(cmux.encode_cmux_frame(0, cmux.SABM, ""))
    uart_write(cmux.encode_cmux_frame(1, cmux.SABM, ""))
    uart_write(cmux.encode_cmux_frame(2, cmux.SABM, ""))
end

--[[
 Parses a single CMUX frame from a byte buffer.
 Returns: DLC number, extracted payload, and remaining buffer (or nils on failure)
--]]
function cmux.parse_cmux_frame(buf)
    local start_idx = buf:find(string.char(cmux.FLAG))
    if not start_idx then
        --gcs:send_text(MAV_SEVERITY.INFO, "no start idx")
        log_data("NOSTART:" .. buf, "{XXX}")
        return nil, nil, nil
    end
    if #buf < 6 then
        return nil, nil, nil, "short"
    end
    local len_byte = buf:byte(4)
    if (len_byte & cmux.EA) == 0 then
        log_data("mux multibyte", "{XXX}")
        return nil, nil, nil -- we don't handle multi-byte length yet
    end
    local len = len_byte >> 1
    local end_idx = 6 + len
    if buf:byte(end_idx) ~= cmux.FLAG then
        log_data("no end idx", "{XXX}")
        return nil, nil, nil, "short"
    end

    local frame = buf:sub(start_idx + 1, end_idx - 1)
    if #frame < 4 then
        log_data("too short", "{XXX}")
        return nil, nil, nil
    end

    local addr = frame:byte(1)
    local ctrl = frame:byte(2)

    --gcs:send_text(MAV_SEVERITY.INFO, string.format("addr=0x%02x ctrl=0x%02x", addr, ctrl))

    if ctrl == cmux.SABM then
        return nil, nil, buf:sub(end_idx + 1)
    end

    if (ctrl & 0xef) ~= cmux.UIH then
        --gcs:send_text(MAV_SEVERITY.INFO, "not UIH")
        return nil, nil, nil
    end

    if #frame ~= 3 + len + 1 then
        log_data("bad flen", "{XXX}")
        return nil, nil, nil
    end

    local data = frame:sub(4, 3 + len)
    local fcs_field = frame:byte(3 + len + 1)
    local header = frame:sub(1, 3)
    local calc_fcs = fcs_calc(header)
    if calc_fcs ~= fcs_field then
        log_data("FCS mismatch", "{XXX}")
        return nil, nil, nil -- FCS mismatch
    end

    local dlc = (addr >> 2) & 0x3F
    local remainder = buf:sub(end_idx + 1)
    --gcs:send_text(MAV_SEVERITY.INFO, string.format("CMUX got: dlc=%d ldata=%d lrem=%d", dlc, #data, #remainder))
    return dlc, data, remainder
end

-- Feeds raw UART data into CMUX frame parser and routes payloads to DLC buffers
function cmux.feed_uart_in(raw)
    while #raw > 0 do
        local dlc, data, rest, err = cmux.parse_cmux_frame(raw)
        if not dlc or not data or not rest then
            if err == "short" then
                -- gcs:send_text(MAV_SEVERITY.INFO, "short")
                return raw
            end
            -- discard
            return ""
        end
        if cmux.buffers[dlc] then
            cmux.buffers[dlc] = cmux.buffers[dlc] .. data
        end
        raw = rest
    end
    return raw
end

--[[
    send data with possible CMUX framing
--]]
local function data_send(data)
    local s
    if cmux_enabled() then
        s = cmux.encode_cmux_frame(cmux.DLC_DATA, cmux.UIH, data)
    else
        s = data
    end
    return uart_write(s) == #s
end

--[[
    send data with possible CMUX framing when connected (logging only if data
    logging enabled)
--]]
local function data_send_connected(data)
    local s
    if cmux_enabled() then
        s = cmux.encode_cmux_frame(cmux.DLC_DATA, cmux.UIH, data)
    else
        s = data
    end
    local n = uart_write(s)
    stats.bytes_out = stats.bytes_out + n
    return n == #s
end

local ati_sequence = 0

local last_data_ms = millis()
local pending_to_modem = ""
local pending_to_fc = ""
local pending_to_parse = ""

--[[
    reset connection buffers
--]]
local function reset_buffers()
    last_data_ms = millis()
    pending_to_modem = ""
    pending_to_fc = ""
    pending_to_parse = ""
    cmux.buffers[cmux.DLC_AT] = ""
    cmux.buffers[cmux.DLC_DATA] = ""
    while ser_device:available() > 0 do
        ser_device:readstring(512)
    end
end

-- reset state and buffers
local function reset_state()
    step = "ATI"
    modem = default_modem
    found_cmux = false
    reset_buffers()
    pending_to_uart = ""
    lte_track.band = nil; lte_track.cid = nil
    cs.cops_rescanning = false; cs.cops_rescan_t   = 0
    cs.qcsq_tries      = 0
    cs.cipopen_retry   = 0;     cs.cipopen_sent    = false; cs.cipopen_sent_ms = 0
    cs.step_times      = {};    cs.step_timer_ms   = millis():tofloat()
end

-- reset back to ATI step
local function reset_to_ATI()
    send_data_reset()
    uart_write_pending()
    reset_state()
end

--[[
    check an ATI response against modem banner strings to auto-detect
    modem type
--]]
local function check_modem_banner(s)
    for model in pairs(modem_list) do
        if s:find(modem_list[model].banner) then
            modem = modem_list[model]
            gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: found modem: " .. model)
            return
        end
    end
end

--[[
    Function to confirm the connection to the modem
    it uses AIT command to get the modem info

    when we enter the ATI step the modem could be in one of several states:

    - in AT command mode
    - in muxed mode
    - in muxed mode at higher baudrate
--]]
local function step_ATI()
    local s = uart_read()
    if s and modem == default_modem then
        check_modem_banner(s)
    end
    if modem ~= default_modem then
        if not cmux_enabled() then
            step = "BAUD"
        else
            step = "CMUX"
        end
        return
    end
    if not option_enabled(LTE_OPTIONS_NOMUX) and s and #s >= 4 and s:byte(1) == cmux.FLAG and s:byte(-1) == cmux.FLAG then
        -- already in mux mode
        found_cmux = true
        gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: in CMUX mode")
        log_data("{INCMUX}", '***')
        AT_send('ATI\r')
        return
    end
    if ati_sequence % 3 == 2 then
        uart_write('+++')   
    elseif ati_sequence % 3 == 1 and not option_enabled(LTE_OPTIONS_NOMUX) then
        uart_write(cmux.encode_cmux_frame(cmux.DLC_AT, cmux.UIH, "ATI\r"))
    else
        uart_write('\rATI\r')
    end
    if ati_sequence % 10 == 5 then
        uart:begin(LTE_BAUD:get())
        log_data(string.format("{BAUD=%d}", LTE_BAUD:get()), '***')
    end
    if ati_sequence % 10 == 9 then
        uart:begin(LTE_IBAUD:get())
        log_data(string.format("{BAUD=%d}", LTE_IBAUD:get()), '***')
    end
    ati_sequence = ati_sequence + 1
end

local change_baud = nil

--[[
    change baud rate
--]]
local function step_BAUD()
    if modem.setbaud and LTE_BAUD:get() ~= LTE_IBAUD:get() then
        change_baud = LTE_BAUD:get()
        AT_send(string.format(modem.setbaud, change_baud))
    end
    step = "CPIN"
end

--[[
    set preferred network using MCC country code and MNC network code
--]]
local function set_MCCMNC()
    if not modem.mccmnc then
        return
    end
    local mccmnc = math.floor(LTE_MCCMNC:get())
    if mccmnc > 0 then
        AT_send(string.format(modem.mccmnc, mccmnc))
    elseif mccmnc == 0 then
        AT_send("AT+COPS=0\r\n")
    end
    last_mccmnc = mccmnc
end

--[[
    set preferred LTE band
--]]
local function set_BAND()
    if not modem.setband and not modem.setband_mask then
        return
    end
    local band = math.floor(LTE_BAND:get())
    if band > 0 then
       if modem.setband_mask then
          AT_send(string.format(modem.setband_mask, 1<<(band-1)))
       else
          AT_send(string.format(modem.setband, band))
       end
    elseif band == 0 then
        AT_send(modem.setband_all)
    end
    last_band = band
end

--[[
    configuration step
--]]
local function step_CONFIG()
    set_BAND()
    set_MCCMNC()
    if modem.config_extra then
       AT_send(modem.config_extra)
    end
    AT_send('AT+CEREG=1\r\n')
    step = "CREG"
end

--[[
    check for a SIM
--]]
local function step_CPIN()
    local s = uart_read()
    if s and s:find("READY") then
        step = "CONFIG"
    end
    AT_send('AT+CPIN?\r\n')
end

--[[
    confirm we are registered (CEREG for EC25 LTE)
--]]
local function step_CREG()
    local s = uart_read()
    if handle_error(s) then return end
    if cs.cops_rescanning then
        if s and (s:find('\r\nOK\r\n') or s:find('\r\nERROR\r\n')) then
            AT_send('AT+COPS=0\r\n'); cs.cops_rescanning = false
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: COPS auto-select')
        elseif cs.cops_rescan_t > 0 and
               millis():tofloat() - cs.cops_rescan_t > 5000 then
            cs.cops_rescanning = false
        end
        return
    end
    if s then
        if cmux_enabled() and #s > 4 and not cmux.parse_cmux_frame(s) then
            step = "CMUX"; return
        end
        local reg = s:match('+CEREG: %d,(%d+)') or s:match('+CREG: %d,(%d+)\r\n')
        if reg == "1" or reg == "5" then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CREG OK')
            step = "QCSQ_GATE"; return
        elseif reg == "0" or reg == "3" then
            gcs:send_text(MAV_SEVERITY.WARNING, 'LTE CREG: not registered — COPS re-scan')
            AT_send('AT+COPS=2\r\n')
            cs.cops_rescanning = true; cs.cops_rescan_t = millis():tofloat(); return
        elseif reg == "2" then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CREG searching...')
        end
    end
    AT_send('AT+CEREG?\r\n')
end

local function step_QCSQ_GATE()
    local s = uart_read()
    if s and s:find('+QCSQ') then
        local rsrp_str = s:match('+QCSQ:%s*"[^"]+",%-?%d+,(%-?%d+)')
        if rsrp_str then
            local rsrp = tonumber(rsrp_str) or -999
            gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE RSRP=%d dBm', rsrp))
            if rsrp >= -100 then
                cs.qcsq_tries = 0
                step = modem.preflight and "QENG" or "QISTATE"
            else
                cs.qcsq_tries = cs.qcsq_tries + 1
                if cs.qcsq_tries <= 3 then
                    gcs:send_text(MAV_SEVERITY.WARNING,
                        string.format('LTE weak RSRP=%d re-scan %d/3', rsrp, cs.qcsq_tries))
                    AT_send('AT+COPS=2\r\n'); AT_send('AT+COPS=0\r\n'); step = "CREG"
                else
                    cs.qcsq_tries = 0
                    gcs:send_text(MAV_SEVERITY.WARNING, 'LTE weak signal — connecting anyway')
                    step = modem.preflight and "QENG" or "QISTATE"
                end
            end
            return
        end
    end
    AT_send('AT+QCSQ\r\n')
end

local function step_QISTATE()
    local s = uart_read()
    if s and #s > 0 then
        if s:find('+QISTATE: 0') then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: socket open, closing first')
            step = "CIPCLOSE"
        else
            step = "CIPOPEN"
        end
        return
    end
    AT_send('AT+QISTATE?\r\n')
end

--[[
    activate network
--]]
local function step_CGACT()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('\r\nOK\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CGACT OK')
        if LTE_PROTOCOL:get() == PPP then
            step = "PPPOPEN"
            last_data_ms = millis()
        else
            step = "CIPMODE"
        end
        return
    end
    data_send(modem.cgact)
    if modem.cfun then
        data_send(modem.cfun)
    end
end

--[[
    set the modem to transparent mode
--]]
local function step_CIPMODE()
    local s = uart_read()
    if s:find('AT+CACID=0,0') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network context set')
        step = "NETOPEN"
        return
    end
    if handle_error(s) then
        return
    end
    if s:find('\r\r\nOK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: transparent mode set')
        step = "NETOPEN"
        return
    end
    data_send(modem.cipmode)
end

--[[
    setup CMUX multiplexing mode
--]]
local function step_CMUX()
    local s = uart_read()
    if s then
        if s:find("CME ERROR") then
            AT_send('AT+CFUN=1\r\n')
        elseif #s >= 4 and (cmux.parse_cmux_frame(s) or
                            s:find('CMUX=0\r\r\nOK\r') or
                            s == string.char(cmux.FLAG,cmux.FLAG,cmux.FLAG,cmux.FLAG)) then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CMUX mode set')
            -- send SABM frames to establish the DLCs
            cmux.send_sabm()
            step = "BAUD"
            return
        end
    end
    uart_write(modem.cmux)
end

--[[
    open the network stack
    needed to be able to open a TCP or UDP connection
--]]
local function step_NETOPEN()
    if not modem.netopen then
        step = "CIPOPEN"
        return
    end
    local s = uart_read()
    if s:find("AT+CNACT=0,1") and s:find("ERROR") and modem.netclose then
        data_send(modem.netclose)
        return
    end
    if handle_error(s) then
        return
    end
    if s and (s:find('NETOPEN\r') or s:find('ACTIVE\r')) and s:find('OK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network opened')
        step = "CIPOPEN"
        return
    end
    data_send(modem.netopen)
end

--[[
    open PPP mode
--]]
local function step_PPPOPEN()
    local s = uart_read()
    if s and modem.cgact and s:find("\r\nNO CARRIER\r\n") then
        send_data_reset()
        step = "ATI"
        return
    end
    if s and s:find("CME ERROR:") then
        send_data_reset()
        step = "ATI"
        return
    end
    if handle_error(s) then
        return
    end

    if s and s:find('CONNECT') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
        reset_buffers()
        step = "CONNECTED"
        return
    end
    data_send(modem.pppopen)
end

--[[
    Close any stale Quectel socket before opening a new one.
    AT+QICLOSE=0 must complete (OK or ERROR) before AT+QIOPEN is sent,
    otherwise the modem returns ERROR on QIOPEN because socket 0 is still
    registered from the previous session.  This step is only entered for
    modems that have a cipclose command defined (EC20, EC25, EC200, BG95,
    EG800Q).  SIM7600 does not define cipclose and skips straight to CIPOPEN.
--]]
local function step_CIPCLOSE()
    local s = uart_read()
    -- any response (OK or ERROR) means the modem has processed the close
    if s and (s:find('\r\nOK\r\n') or s:find('\nERROR\r\n') or s:find('+QICLOSE')) then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: socket closed, opening')
        step = "CIPOPEN"
        return
    end
    AT_send(modem.cipclose)
end

--[[
    open a TCP or UDP connection to the server
    the server IP and port are defined in the parameters
--]]
local function step_CIPOPEN()
    local s = uart_read()
    if handle_error(s) then return end
    if s then
        if s:find('+CAOPEN: 0,0') and s:find('OK\r\n') and modem.caswitch then
            data_send(modem.caswitch); return
        end
        if s:find('CONNECT') or s:find('+QIOPEN: 0,0') or
           (s:find('+CAOPEN: 0,0') and s:find('OK\r\n')) then
            gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
            cs.cipopen_sent = false; cs.cipopen_retry = 0
            reset_buffers(); step = "CONNECTED"; return
        end
    end
    if cs.cipopen_sent then
        local wait = cs.cipopen_sent_ms > 0 and
                     millis():tofloat() - cs.cipopen_sent_ms or 0
        if wait > 3000 then
            cs.cipopen_retry = cs.cipopen_retry + 1
            cs.cipopen_sent  = false
            if cs.cipopen_retry <= 3 then
                gcs:send_text(MAV_SEVERITY.WARNING,
                    string.format('LTE CIPOPEN timeout retry %d/3', cs.cipopen_retry))
                if modem.cipclose then AT_send(modem.cipclose) end
            else
                gcs:send_text(MAV_SEVERITY.ERROR, 'LTE CIPOPEN failed 3x — reset')
                cs.cipopen_retry = 0; reset_to_ATI()
            end
        end
        return
    end
    if LTE_SERVER_PORT:get() <= 0 then
        gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT"); return
    end
    local cipopen = option_enabled(LTE_OPTIONS_TCP) and
                    modem.cipopen_tcp or modem.cipopen_udp
    data_send(string.format(cipopen,
        LTE_SERVER_IP0:get(), LTE_SERVER_IP1:get(),
        LTE_SERVER_IP2:get(), LTE_SERVER_IP3:get(),
        LTE_SERVER_PORT:get()))
    cs.cipopen_sent = true; cs.cipopen_sent_ms = millis():tofloat()
end

--[[
    check for CSQ reply
--]]
local function check_CSQ(s)
    local rssi_raw, ber_raw = s:match("%+CSQ:%s*(%d+),(%d+)")
    if rssi_raw then
        gcs:send_named_float('LTE_RSSI', rssi_raw)
        logger:write("LTE",'RSSI,BER,Bin,Bout','iiII',
                     rssi_raw,
                     ber_raw,
                     stats.bytes_in,
                     stats.bytes_out)
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("RSSI:%d BER:%d", rssi_raw, ber_raw))
        return true
    end
    return false
end

--[[
    check for CGACT reply
--]]
local function check_CGACT(s)
    local ctx, active = s:match("%+CGACT:%s*(%d+),(%d+)")
    if ctx then
        ctx = tonumber(ctx) or 0
        active = tonumber(active) or 0
        gcs:send_text(MAV_SEVERITY.INFO, string.format("CGACT: %d,%d", ctx, active))
        return true
    end
    return false
end

--[[
    check for CPSI reply
--]]
local function check_CPSI(s)
    -- example1: +CPSI: LTE,Online,505-02,0xCBE8,36519691,101,EUTRAN-BAND3,1800,5,5,-147,-1143,-764,11
    -- example2: +CPSI: LTE CAT-M1,Online,505-01,0x2036,134523149,238,EUTRAN-BAND28,9410,5,5,-20,-116,-82,6

    if not s:find("+CPSI") then
        return false
    end

    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))

    local system_mode, operation_mode, mcc_mnc, tac_str, scell_id_str, pcid_str, earfcn_band, ul_freq_str, dl_freq_str, tdd_cfg_str, rsrq_str, rsrp_str, rssi_str, sinr_str =
    s:match("+CPSI:%s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([%-]?%d+),([%-]?%d+),([%-]?%d+),([%-]?%d+)")

    if system_mode and sinr_str then
        -- Convert strings to numbers
        local tac = tonumber(tac_str:match("0x(%w+)"), 16) or tonumber(tac_str) or 0
        local scell_id = tonumber(scell_id_str) or 0
        local pcid = tonumber(pcid_str) or 0
        local ul_freq = tonumber(ul_freq_str) or 0
        local dl_freq = tonumber(dl_freq_str) or 0
        local tdd_cfg = tonumber(tdd_cfg_str) or 0
        local rsrp = tonumber(rsrp_str) or 0
        local rsrq = tonumber(rsrq_str) or 0
        local rssi = tonumber(rssi_str) or 0
        local sinr = tonumber(sinr_str) or 0
        local band = earfcn_band:match("[^%d]+(%d+)") or -1
        logger:write("LTES",'Md,Op,MCC,TAC,CID,PID,BND,F,DF,TDD,RP,RQ,RS,SR','NNNIIINHhhhhhh',
                     system_mode, operation_mode, mcc_mnc, tac, scell_id, pcid, earfcn_band,
                     ul_freq, dl_freq, tdd_cfg, rsrp, rsrq, rssi, sinr)
        if option_enabled(LTE_OPTIONS_SIGNALS) then
            gcs:send_named_float('LTE_RSRP', rsrp)
            gcs:send_named_float('LTE_RSRQ', rsrq)
            gcs:send_named_float('LTE_BAND', band)
            -- shift to remove antenna selection within tower
            gcs:send_named_float('LTE_CID', scell_id>>8)
            local mcc, mnc = mcc_mnc:match("(%d+)-(%d+)")
            if mcc and mnc then
                gcs:send_named_float('LTE_MCCMNC', mcc*100+mnc)
            end
        end

        -- GCS band / cell-tower announcements (always active, not gated on OPTIONS)
        local band_num = tonumber(band) or -1
        local tower_id = scell_id >> 8   -- strip antenna selection bits

        -- first time: announce current band
        if lte_track.band == nil then
            gcs:send_text(MAV_SEVERITY.INFO,
                string.format("LTE: connected on Band %s (%s)", tostring(band), earfcn_band))
            lte_track.band = band_num
        end
        -- first time: record tower
        if lte_track.cid == nil then
            lte_track.cid = tower_id
        end

        -- band switch warning
        if lte_track.band ~= nil and band_num ~= lte_track.band then
            gcs:send_text(MAV_SEVERITY.WARNING,
                string.format("LTE WARNING: band switch %d -> %d (%s)",
                              lte_track.band, band_num, earfcn_band))
            lte_track.band = band_num
        end
        -- cell tower switch warning
        if lte_track.cid ~= nil and tower_id ~= lte_track.cid then
            gcs:send_text(MAV_SEVERITY.WARNING,
                string.format("LTE WARNING: cell tower switch CID %d -> %d",
                              lte_track.cid, tower_id))
            lte_track.cid = tower_id
        end

        return true
    end
    return false
end

--[[
    check for QENG reply
--]]
local function check_QENG(s)
    if not s:find('+QENG:%s*"servingcell"') then
        return false
    end

    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))

    local data_str = s:match('%+QENG:%s*"servingcell",(.*)')
    if not data_str then return false end

    data_str = data_str:gsub('"', '')

    local t = {}
    for part in string.gmatch(data_str, "([^,\r\n]+)") do
        table.insert(t, part)
    end

    if t[2] == "LTE" then
        local is_fdd = (t[3] == "FDD")
        local tac_idx = is_fdd and 12 or 11

        local mcc    = tonumber(t[4]) or 0
        local mnc    = tonumber(t[5]) or 0
        local cid    = tonumber(t[6], 16) or 0
        local pcid   = tonumber(t[7]) or 0
        local earfcn = tonumber(t[8]) or 0
        local band   = tonumber(t[9]) or -1
        local tac    = tonumber(t[tac_idx], 16) or 0
        local rsrp   = tonumber(t[tac_idx+1]) or 0
        local rsrq   = tonumber(t[tac_idx+2]) or 0
        local rssi   = tonumber(t[tac_idx+3]) or 0
        local sinr   = tonumber(t[tac_idx+4]) or 0

        logger:write("LTES", 'MCC,MNC,TAC,CID,PID,EF,RSRP,RSRQ,RSSI,SINR', 'iiiiiiiiii',
                     mcc, mnc, tac, cid, pcid, earfcn, rsrp, rsrq, rssi, sinr)

        if option_enabled(LTE_OPTIONS_SIGNALS) then
            gcs:send_named_float('LTE_RSRP', rsrp)
            gcs:send_named_float('LTE_RSRQ', rsrq)
            gcs:send_named_float('LTE_SINR', sinr)
            gcs:send_named_float('LTE_BAND', band)
            gcs:send_named_float('LTE_CID', cid>>8)
            gcs:send_named_float('LTE_MCCMNC', mcc*100+mnc)
        end

        local tower_id = cid >> 8

        if lte_track.band == nil then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE: connected on Band %d (EARFCN %d)", band, earfcn))
            lte_track.band = band
        end
        if lte_track.cid == nil then
            lte_track.cid = tower_id
        end

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

--[[
    handle AT replies in CMUX mode
--]]
local function handle_AT_reply(s)

    --gcs:send_text(MAV_SEVERITY.INFO, "AT REPLY: " .. s:gsub('\r', ''):gsub('\n', ' '))
    check_CSQ(s)
    if check_CPSI(s) then
        return
    end
    if check_QENG(s) then
        return
    end
    if check_CGACT(s) then
        return
    end

    if s:find("PPPD: DISCONNECTED") then
        step = "PPPOPEN"
    end
end

local last_CSQ_ms = millis()
local last_CSQ_reply_ms = uint32_t(0)
local last_parse_ms = uint32_t(0)
local last_route_ms = uint32_t(0)
local last_send_data_ms = uint32_t(0)
local csq_toggle = false
--[[
    handle data while connected
--]]
local function step_CONNECTED()
    local s = uart:readstring(512) or ""
    stats.bytes_in = stats.bytes_in + #s
    if option_enabled(LTE_OPTIONS_LOGALL) then
        log_data(s, '<<<')
    end
    if s and s:find('\r\nCLOSED\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connection closed, reconnecting')
        cs.cipopen_sent = false; step = "CIPOPEN"; return
    end
    if s and (s:find('+CEREG: 0') or s:find('+CREG: 0,0') or s:find('+CREG: 0,2')) then
        gcs:send_text(MAV_SEVERITY.WARNING, 'LTE: network lost — fast reconnect')
        cs.cipopen_sent = false
        if modem.cipclose then AT_send(modem.cipclose) end
        step = "CREG"; return
    end
    if s and s:find('PPPD: DISCONNECTED\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: PPP closed, reconnecting')
        step = "PPPOPEN"
        return
    end
    local now_ms = millis()
    if s and #s > 0 then
        if not cmux_enabled() then
            pending_to_fc = pending_to_fc .. s
            last_data_ms = now_ms
        else
            pending_to_parse = pending_to_parse .. s
            pending_to_parse = cmux.feed_uart_in(pending_to_parse)
            if now_ms - last_parse_ms > 1000 then
                pending_to_parse = ""
            end
            if #cmux.buffers[cmux.DLC_AT] > 0 then
                last_parse_ms = now_ms
                --gcs:send_text(MAV_SEVERITY.INFO, string.format("AT reply %d", #cmux.buffers[cmux.DLC_AT]))
                handle_AT_reply(cmux.buffers[cmux.DLC_AT])
                cmux.buffers[cmux.DLC_AT] = ""
            end
            if #cmux.buffers[cmux.DLC_DATA] > 0 then
                last_data_ms = now_ms
                -- gcs:send_text(MAV_SEVERITY.INFO, string.format("data input %d", #cmux.buffers[cmux.DLC_DATA]))
                last_parse_ms = now_ms
                pending_to_fc = pending_to_fc .. cmux.buffers[cmux.DLC_DATA]
                cmux.buffers[cmux.DLC_DATA] = ""
            end
        end
    elseif LTE_TIMEOUT:get() > 0 and now_ms - last_data_ms > uint32_t(LTE_TIMEOUT:get() * 1000) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: timeout')
        reset_to_ATI()
        return
    end
    s = ser_device:readstring(512)
    if s then
        pending_to_modem = pending_to_modem .. s
    end

    --[[
        going via these pending buffers allows for rapid bursts of data and takes advantage
        of the hardware flow control
    --]]
    local buffer_limit = 10240 -- so we don't run out of memory
    if #pending_to_modem > buffer_limit then
        pending_to_modem = ""
    end
    if #pending_to_fc > buffer_limit then
        pending_to_fc = ""
    end

    local quota = 0
    if LTE_TX_RATE:get() > 0 then
        local dt = (now_ms - last_send_data_ms):tofloat()*0.001
        quota = math.floor(dt * LTE_TX_RATE:get())
    end

    local data_sent = 0
    while #pending_to_modem > 0 do
        local n = #pending_to_modem
        if n > 100 then
            n = 100
        end
        if quota > 0 and quota - data_sent < n then
            n = quota - data_sent
        end
        local data = pending_to_modem:sub(1, n)

        data_sent = data_sent + #data
        last_send_data_ms = now_ms

        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("data output %d", n))
        if not data_send_connected(data) then
            break
        end
        pending_to_modem = pending_to_modem:sub(n + 1)

        if quota > 0 and data_sent >= quota then
            break
        end
    end
    if #pending_to_fc > 0 then
        local nwritten = ser_device:writestring(pending_to_fc)
        if nwritten > 0 then
            pending_to_fc = pending_to_fc:sub(nwritten + 1)
        end
    end
    if cmux_enabled() and not option_enabled(LTE_OPTIONS_NOSIGQUERY) then
        -- Ping-pong between CSQ and QENG every 500ms to avoid overwhelming the modem AT parser
        if now_ms - last_CSQ_ms > 500 then
            last_CSQ_ms = now_ms
            if not modem.cpsi then
                AT_send("AT+CSQ\r\n") -- Fallback for modems without QENG
            else
                if csq_toggle then
                    AT_send("AT+CSQ\r\n")
                else
                    AT_send(modem.cpsi)
                end
                csq_toggle = not csq_toggle
            end
        end

        if now_ms - last_CSQ_reply_ms > 5000 then
            last_CSQ_reply_ms = now_ms
            gcs:send_named_float('LTE_RSSI', -1)
        end
        if LTE_MCCMNC:get() ~= last_mccmnc and modem.mccmnc then
            set_MCCMNC()
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE_modem: set MCCMNC=%d", last_mccmnc))
            step = "CREG"
        end
        if LTE_BAND:get() ~= last_band and (modem.setband or modem.setband_mask) then
            set_BAND()
            if last_band ~= 0 then
                step = "CREG"
            end
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE_modem: set BAND=%d", last_band))
        end
    end

    -- newer firmware allows for multiple PPP interfaces and custom routing
    if supports_routing and now_ms - last_route_ms > 1000 then
        last_route_ms = now_ms
        local dest = uint32_t(LTE_ROUTE_IP0:get())<<24
        dest = dest | uint32_t(LTE_ROUTE_IP1:get())<<16
        dest = dest | uint32_t(LTE_ROUTE_IP2:get())<<8
        dest = dest | uint32_t(LTE_ROUTE_IP3:get())
        if dest ~= uint32_t(0) then
            networking:add_route(0, 1, dest, math.floor(LTE_ROUTE_MASK:get())) -- luacheck: ignore 143
        end
    end
end


local function run_step()
    if change_baud then
        uart:begin(change_baud)
        change_baud = nil
    end

    -- detect step transitions BEFORE CONNECTED early return
    if cs.last_step ~= step then
        if cs.last_step and cs.last_step ~= "ATI" then
            cs.step_times = cs.step_times or {}
            table.insert(cs.step_times,
                {name=cs.last_step, ms=math.floor(millis():tofloat()-cs.step_timer_ms)})
        end
        cs.step_timer_ms = millis():tofloat()
        if step == "CONNECTED" and cs.step_times and #cs.step_times > 0 then
            local parts, total = {}, 0
            for _, t in ipairs(cs.step_times) do
                table.insert(parts, t.name..':'..t.ms..'ms'); total = total + t.ms
            end
            gcs:send_text(MAV_SEVERITY.INFO,
                'LTE timing: '..table.concat(parts,' ')..' total:'..total..'ms')
            cs.step_times = {}
        end
    end
    cs.last_step = step

    if step == "CONNECTED" then
        step_CONNECTED(); cs.step_count = 0; return 5
    end

    -- prevent getting stuck
    if step == cs.last_step and step ~= "ATI" then
        cs.step_count = cs.step_count + 1
        if cs.step_count > 50 then
            gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: step reset"); reset_to_ATI()
        end
    else
        cs.step_count = 0
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE_modem: step %s', step))

    if step == "ATI" then
        step_ATI()
        return 1100
    end

    if step == "BAUD" then
        step_BAUD()
        return 50
    end

    if step == "CREG" then
        step_CREG()
        return 50
    end

    if step == "QCSQ_GATE" then
        step_QCSQ_GATE()
        return 50
    end

    if step == "QISTATE" then
        step_QISTATE()
        return 50
    end

    if step == "CGACT" then
        step_CGACT()
        return 500
    end
    
    if step == "CIPMODE" then
        step_CIPMODE()
        return 200
    end

    if step == "NETOPEN" then
        step_NETOPEN()
        return 200
    end

    if step == "CONFIG" then
        step_CONFIG()
        return 50
    end
    
    if step == "CMUX" then
        step_CMUX()
        return 50
    end

    if step == "CPIN" then
        step_CPIN()
        return 50
    end
    
    if step == "PPPOPEN" then
        step_PPPOPEN()
        return 200
    end
    
    if step == "CIPCLOSE" then
        step_CIPCLOSE()
        return 50
    end

    if step == "QENG" then
        step_QENG()
        return 50
    end

    if step == "CIPOPEN" then
        step_CIPOPEN()
        return 50
    end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("LTE_modem: bad step %s", step))
    reset_to_ATI()
end

local function update()
    if LTE_ENABLE:get() == 0 then
        return 500
    end
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