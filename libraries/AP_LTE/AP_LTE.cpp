/*
  LTE modem driver, ported from the LTE_modem.lua applet.

  Transparent-mode socket only. The applet's CMUX, direct-push (QISEND) and
  PPP transports are deliberately absent -- see AP_LTE.h.
 */

#include "AP_LTE.h"

#if AP_LTE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include <stdarg.h>

extern const AP_HAL::HAL &hal;

AP_LTE *AP_LTE::_singleton;

// Bytes queued for the modem UART. Must hold the whole CA certificate, which
// the HTTPAUTH cert upload hands over in a single write.
#define AP_LTE_TXQ_SIZE 2560

const AP_Param::GroupInfo AP_LTE::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable LTE modem driver
    // @Description: Enable the LTE modem driver. Takes effect at boot.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_LTE, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: SERPORT
    // @DisplayName: Modem serial port
    // @Description: Which serial port the modem is attached to, as an instance index among the ports with SERIALn_PROTOCOL set to 28 (Scripting). Unchanged from the Lua applet so existing wiring keeps working.
    // @Range: 0 5
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SERPORT", 2, AP_LTE, _serport, 0),

    // @Param: SCRPORT
    // @DisplayName: Unused
    // @Description: Retained so the parameter index is not reused. The Lua applet used this to pick a SCR_SDEV simulated device; this driver owns its virtual port outright and does not need one.
    // @User: Advanced
    AP_GROUPINFO("SCRPORT", 3, AP_LTE, _scrport, 0),

    // @Param: SERVER_IP0
    // @DisplayName: Server IP octet 0
    // @Description: First octet of the relay server address, used only when LTE_HTTPAUTH is 0.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("SERVER_IP0", 4, AP_LTE, _server_ip0, 0),

    // @Param: SERVER_IP1
    // @DisplayName: Server IP octet 1
    // @Description: Second octet of the relay server address.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("SERVER_IP1", 5, AP_LTE, _server_ip1, 0),

    // @Param: SERVER_IP2
    // @DisplayName: Server IP octet 2
    // @Description: Third octet of the relay server address.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("SERVER_IP2", 6, AP_LTE, _server_ip2, 0),

    // @Param: SERVER_IP3
    // @DisplayName: Server IP octet 3
    // @Description: Fourth octet of the relay server address.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("SERVER_IP3", 7, AP_LTE, _server_ip3, 0),

    // @Param: SERVER_PORT
    // @DisplayName: Server port
    // @Description: Relay server port, used only when LTE_HTTPAUTH is 0.
    // @Range: 0 65535
    // @User: Standard
    AP_GROUPINFO("SERVER_PORT", 8, AP_LTE, _server_port, 0),

    // @Param: BAUD
    // @DisplayName: Modem operating baud rate
    // @Description: Baud rate the modem is switched to after boot, on families that support AT+IPR.
    // @User: Standard
    AP_GROUPINFO("BAUD", 9, AP_LTE, _baud, 115200),

    // @Param: TIMEOUT
    // @DisplayName: Data timeout
    // @Description: Seconds without any data from the modem while connected before the link is declared dead and the modem is reset. 0 disables.
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("TIMEOUT", 10, AP_LTE, _timeout, 30),

    // @Param: PROTOCOL
    // @DisplayName: Virtual port protocol
    // @Description: Serial protocol presented on the virtual port the modem link carries. 2 is MAVLink2.
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PROTOCOL", 11, AP_LTE, _protocol, 2),

    // @Param: OPTIONS
    // @DisplayName: Driver options
    // @Description: Option bits. DirectPush is inert in this driver, which does not have that transport, but keeps its bit position so a value carried over from the Lua applet still means the same thing.
    // @Bitmask: 0:LogAll,1:Signals,2:NoMux,3:NoSigQuery,4:TCP,5:DirectPush(inert)
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 12, AP_LTE, _options, 6),

    // @Param: IBAUD
    // @DisplayName: Modem initial baud rate
    // @Description: Baud rate used to first talk to the modem, before LTE_BAUD is applied.
    // @User: Advanced
    AP_GROUPINFO("IBAUD", 13, AP_LTE, _ibaud, 115200),

    // @Param: MCCMNC
    // @DisplayName: Operator lock
    // @Description: Lock to this MCCMNC. -1 leaves operator selection alone, 0 forces automatic selection.
    // @User: Advanced
    AP_GROUPINFO("MCCMNC", 14, AP_LTE, _mccmnc, -1),

    // indices 15-19 were LTE_ROUTE_IP0-3 and LTE_ROUTE_MASK, which needed
    // networking:add_route. That binding does not exist in this tree, so the
    // applet had them permanently disabled. Indices 24 and 25 were
    // LTE_TX_DEAD and LTE_SOCK_T, both direct-push only. All five are left
    // free rather than reused, so a downgrade to the applet still lines up.

    // @Param: TX_RATE
    // @DisplayName: Uplink rate limit
    // @Description: Maximum uplink bytes per second toward the modem. 0 is unlimited.
    // @Units: B/s
    // @User: Advanced
    AP_GROUPINFO("TX_RATE", 20, AP_LTE, _tx_rate, 0),

    // @Param: BAND
    // @DisplayName: Band lock
    // @Description: Lock to this LTE band number. -1 leaves band selection alone, 0 enables all bands.
    // @User: Advanced
    AP_GROUPINFO("BAND", 21, AP_LTE, _band, -1),

    // @Param: GRACE
    // @DisplayName: Registration loss grace
    // @Description: Seconds a registration drop is tolerated while connected before reconnecting.
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("GRACE", 22, AP_LTE, _grace, 3),

    // @Param: STUCK_T
    // @DisplayName: Step timeout
    // @Description: Seconds a setup step may sit without progress before the modem is hard reset.
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("STUCK_T", 23, AP_LTE, _stuck_t, 15),

    // @Param: HTTPAUTH
    // @DisplayName: Fetch server address over HTTPS
    // @Description: Fetch the relay server address from the auth endpoint over TLS with full certificate verification before opening the socket. There is no static fallback: a failed login halts the driver.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HTTPAUTH", 26, AP_LTE, _httpauth, 1),

    // @Param: AUTHSYS
    // @DisplayName: Apply sysId from auth response
    // @Description: Apply the sysId the auth server returns, saving it to SYSID_THISMAV.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTHSYS", 27, AP_LTE, _authsys, 1),

    // @Param: AUTHKEY
    // @DisplayName: Apply MAVLink signing key from auth response
    // @Description: Apply the signing key the auth server returns. Setting this to 0 does not merely stop applying it, it erases the stored key, which otherwise persists in FRAM and is re-armed at every boot.
    // @Values: 0:Disabled and key erased,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTHKEY", 28, AP_LTE, _authkey, 0),

    AP_GROUPEND
};

AP_LTE::AP_LTE()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_LTE must be singleton");
    }
    _singleton = this;

    _modem = &AP_LTE_default_modem;
    _step = Step::ATI;
    // Any value but ATI, so the first pass counts as a step change and prints
    // "step ATI". _reset_recorded suppresses the timing entry that change
    // would otherwise attribute to this non-existent previous step.
    _last_step = Step::HALT;
    _reset_recorded = true;
    _authkey_seen = -1;
    _track_band = -1;
    _track_cid = -1;
    _last_mccmnc = INT32_MIN;
    _last_band = INT32_MIN;
    _log_fd = -1;
    _post_reset = true;
    _found_cmux = false;
    _cmux_was_set = false;
    _cmux_force_disabled = false;
    _cmux_cpin_strikes = 0;
}

// ---------------------------------------------------------------------------
// startup
// ---------------------------------------------------------------------------

void AP_LTE::pre_init()
{
    if (pget(_enable) == 0) {
        return;
    }
    if (!_port.init_buffers(AP_LTE_VPORT_RXBUF, AP_LTE_VPORT_TXBUF)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: no memory for port");
        return;
    }
    _port.state.idx = AP_SERIALMANAGER_LTE_PORT;
    _port.state.protocol.set(pget(_protocol));
    _port.state.baud.set(pget(_baud));
    _port.state.options.set(0);
    AP::serialmanager().register_port(&_port);
    _port_registered = true;
}

void AP_LTE::init()
{
    // The thread runs even when disabled: LTE_AUTHKEY=0 has to be able to
    // erase a stored signing key, and signing stuck on with the driver off is
    // the one state with no way out.
    if (pget(_enable) != 0) {
        if (!_port_registered) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: port not registered");
            return;
        }
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Scripting,
                                                pget(_serport));
        if (_uart == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: could not find serial port");
            return;
        }
        _txq = NEW_NOTHROW ByteBuffer(AP_LTE_TXQ_SIZE);
        if (_txq == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: no memory");
            return;
        }
        _uart->begin(pget(_ibaud));
        log_open();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: starting");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_LTE::thread_main, void),
                                      "LTE", 4096, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: failed to start thread");
    }
}

/*
  Driver loop. Mirrors update() in the applet: the signing-key erase runs
  ahead of the enable check on purpose, then the step machine chooses its own
  delay for the next pass.
 */
void AP_LTE::thread_main()
{
    while (true) {
        // AUTHKEY == 0 means signing off, not just "do not apply a key": the
        // key lives in FRAM, not a parameter, so it is re-armed every boot
        // until the block is erased. Key and timestamp must both be zeroed to
        // trip the all-zero branch in load_signing_key(). Edge triggered and
        // latched only on success, since set_signing_key() refuses while armed.
        const int8_t authkey = int8_t(pget(_authkey));
        if (authkey != _authkey_seen) {
            if (authkey != 0) {
                _authkey_seen = authkey;
            } else {
                const uint8_t zero_key[32] {};
                if (gcs().set_signing_key(zero_key, 0)) {
                    _authkey_seen = authkey;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "LTE: LTE_AUTHKEY=0, MAVLink signing key erased");
                }
            }
        }

        if (pget(_enable) == 0 || _uart == nullptr) {
            hal.scheduler->delay(500);
            continue;
        }

        // Modem and SIM identity into the dataflash log. SIMINFO clears
        // _ltei_ms so a row goes out at every registration; the 0.1Hz repeat
        // covers rows lost to a full logger buffer, which is likely at
        // registration while formats and params are still streaming.
        if (_sim_name[0] != 0) {
            const uint32_t now = AP_HAL::millis();
            if (_ltei_ms == 0 || now - _ltei_ms > 10000) {
                _ltei_ms = now;
                // The units/mults arguments are not decorative here. Write()
                // is overloaded on (name, labels, fmt, ...) and (name, labels,
                // units, mults, fmt, ...), and when the leading data arguments
                // are themselves strings the five-string overload wins
                // resolution silently -- the firmware string lands in fmt and
                // the logger panics on it at runtime. Passing units and mults
                // explicitly makes the intended overload the only one.
                AP::logger().Write("LTEI", "Mdl,FW,Sim,Net", "----", "----", "NNNN",
                                   _modem->name,
                                   _fw_short[0] != 0 ? _fw_short :
                                   (_modem_revision[0] != 0 ? _modem_revision : "unknown"),
                                   _sim_name,
                                   _sim_oper[0] != 0 ? _sim_oper : "unknown");
            }
        }

        const uint32_t delay_ms = run_step();
        uart_write_pending();
        hal.scheduler->delay(delay_ms);
    }
}

// ---------------------------------------------------------------------------
// step dispatch
// ---------------------------------------------------------------------------

const char *AP_LTE::step_name(Step s)
{
    switch (s) {
    case Step::ATI:          return "ATI";
    case Step::CMUX:         return "CMUX";
    case Step::BAUD:         return "BAUD";
    case Step::CPIN:         return "CPIN";
    case Step::CONFIG:       return "CONFIG";
    case Step::CREG:         return "CREG";
    case Step::SIMINFO:      return "SIMINFO";
    case Step::HTTPAUTH:     return "HTTPAUTH";
    case Step::SIGNAL_GATE:  return "SIGNAL_GATE";
    case Step::QENG:         return "QENG";
    case Step::CEREG_CHECK:  return "CEREG_CHECK";
    case Step::SOCKET_STATE: return "SOCKET_STATE";
    case Step::CIPCLOSE:     return "CIPCLOSE";
    case Step::CGACT:        return "CGACT";
    case Step::PPPOPEN:      return "PPPOPEN";
    case Step::CIPMODE:      return "CIPMODE";
    case Step::NETOPEN:      return "NETOPEN";
    case Step::CIPOPEN:      return "CIPOPEN";
    case Step::CONNECTED:    return "CONNECTED";
    case Step::HALT:         return "HALT";
    }
    return "?";
}

uint32_t AP_LTE::run_step()
{
    if (_change_baud != 0) {
        _uart->begin(_change_baud);
        _change_baud = 0;
    }

    const bool step_changed = (_last_step != _step);
    const uint32_t now_ms = AP_HAL::millis();

    if (step_changed) {
        // SIMINFO is silent: it announces itself with the module/firmware/sim
        // lines it prints, so a step line ahead of them says nothing extra.
        if (_step != Step::SIMINFO) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: step %s", step_name(_step));
        }
        if (!_reset_recorded && _last_step != Step::ATI &&
            _step_times_n < ARRAY_SIZE(_step_times)) {
            _step_times[_step_times_n].name = step_name(_last_step);
            _step_times[_step_times_n].ms = now_ms - _step_timer_ms;
            _step_times_n++;
        }
        _reset_recorded = false;
        _step_timer_ms = now_ms;

        if (_step == Step::CIPMODE) {
            // Gates the "bare OK means transparent mode is set" check below, so
            // a reply still in flight from an earlier fire-and-forget command
            // cannot be mistaken for the CIPMODE reply.
            _cipmode_sent = false;
        }

        if (_step == Step::CONNECTED && _step_times_n > 0) {
            _post_reset = false;
            char line[128] {};
            uint32_t total = 0;
            const char *max_name = "none";
            uint32_t max_ms = 0;
            uint8_t used = 0;
            for (uint8_t i = 0; i < _step_times_n; i++) {
                total += _step_times[i].ms;
                if (_step_times[i].ms > max_ms) {
                    max_ms = _step_times[i].ms;
                    max_name = _step_times[i].name;
                }
                used += hal.util->snprintf(&line[used], sizeof(line) - used, "%s:%lums ",
                                           _step_times[i].name,
                                           (unsigned long)_step_times[i].ms);
                if (used >= sizeof(line) - 1) {
                    break;
                }
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE longest step: %s (%lums)",
                          max_name, (unsigned long)max_ms);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE timing: %s total:%lums",
                          line, (unsigned long)total);
            if (_disconnect_ms != 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: total outage %.1fs",
                              (double)(now_ms - _disconnect_ms) * 0.001);
                _disconnect_ms = 0;
            }
            _step_times_n = 0;
        }
    }
    _last_step = _step;

    // 20ms, not 5ms: matches the HAL UART buffer sizing assumption of a ~40Hz
    // reader (AP_HAL_ChibiOS/UARTDriver.cpp).
    if (_step == Step::CONNECTED) {
        step_CONNECTED();
        return 20;
    }

    const uint32_t time_in_step_s = (now_ms - _step_timer_ms) / 1000;

    if (!step_changed && _step != Step::ATI && _step != Step::CMUX &&
        _step != Step::CPIN && _step != Step::CREG && _step != Step::HALT &&
        _step != Step::HTTPAUTH) {
        if (time_in_step_s > uint32_t(pget(_stuck_t))) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: %s timeout after %ds",
                          step_name(_step), int(pget(_stuck_t)));
            reset_to_ATI();
            return 1000;
        }
    }

    if (!step_changed && (_step == Step::CPIN || _step == Step::CREG ||
                          _step == Step::CMUX)) {
        const uint32_t patience = MAX(30U, uint32_t(pget(_stuck_t)) * 3);
        if (time_in_step_s > patience) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LTE: %s search failed, hard reset",
                          step_name(_step));
            reset_to_ATI();
            return 1000;
        }
    }

    switch (_step) {
    case Step::HALT:
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LTE HALTED: %s. Power cycle modem.",
                      _halt_reason != nullptr ? _halt_reason : "unknown reason");
        return 15000;
    case Step::ATI:          step_ATI();          return 1100;
    case Step::CMUX:         step_CMUX();         return 50;
    case Step::BAUD:         step_BAUD();         return 50;
    case Step::CPIN:         step_CPIN();         return 50;
    case Step::CONFIG:       step_CONFIG();       return 50;
    case Step::CREG:         step_CREG();         return 150;
    case Step::SIMINFO:      step_SIMINFO();      return 50;
    case Step::HTTPAUTH:     step_HTTPAUTH();     return 100;
    case Step::SIGNAL_GATE:  step_SIGNAL_GATE();  return 50;
    case Step::QENG:         step_QENG();         return 50;
    case Step::CEREG_CHECK:  step_CEREG_CHECK();  return 100;
    case Step::SOCKET_STATE: step_SOCKET_STATE(); return 50;
    case Step::CIPCLOSE:     step_CIPCLOSE();     return 50;
    case Step::CGACT:        step_CGACT();        return 200;
    case Step::PPPOPEN:      step_PPPOPEN();      return 200;
    case Step::CIPMODE:      step_CIPMODE();      return 200;
    case Step::NETOPEN:      step_NETOPEN();      return 200;
    case Step::CIPOPEN:      step_CIPOPEN();      return 50;
    case Step::CONNECTED:                         return 20;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: bad step");
    reset_to_ATI();
    return 1000;
}

bool AP_LTE::ppp_mode() const
{
    return uint8_t(pget(_protocol)) == AP_SerialManager::SerialProtocol_PPP;
}

AP_LTE::Step AP_LTE::ppp_entry() const
{
    return _modem->cgact != nullptr ? Step::CGACT : Step::PPPOPEN;
}

AP_LTE::Step AP_LTE::next_after_registration() const
{
    if (ppp_mode()) {
        return ppp_entry();
    }
    if (_modem->fast_connect) {
        return Step::SIGNAL_GATE;
    }
    if (_modem->preflight != nullptr) {
        return Step::QENG;
    }
    if (_modem->cipmode != nullptr) {
        return Step::CIPMODE;
    }
    if (_modem->cipclose != nullptr) {
        return Step::CIPCLOSE;
    }
    return Step::CIPOPEN;
}

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------

// Bytes exactly as the modem sent them, no de-framing. Used only by the steps
// that run before the mux exists, where there is nothing to de-frame.
uint16_t AP_LTE::uart_read_raw()
{
    _raw_len = 0;
    if (_uart == nullptr) {
        return 0;
    }
    const ssize_t n = _uart->read(_raw, sizeof(_raw));
    if (n <= 0) {
        return 0;
    }
    _raw_len = uint16_t(n);
    log_raw(_raw, _raw_len, "<<<");
    _bytes_in += _raw_len;
    if (_step != Step::CONNECTED) {
        _last_data_ms = AP_HAL::millis();
    }
    return _raw_len;
}

uint16_t AP_LTE::uart_read()
{
    _chunk_len = 0;
    _dlc2_len = 0;
    if (uart_read_raw() == 0) {
        return 0;
    }
    if (!cmux_enabled()) {
        memcpy(_chunk, _raw, _raw_len);
        _chunk_len = _raw_len;
        return _chunk_len;
    }
    // fills _chunk from DLC 1 and _dlc2 from DLC 2
    cmux_feed(_raw, _raw_len);
    return _chunk_len;
}

void AP_LTE::uart_write_raw(const char *s)
{
    if (s == nullptr || _txq == nullptr) {
        return;
    }
    const uint16_t len = uint16_t(strlen(s));
    _txq->write((const uint8_t *)s, len);
    log_raw((const uint8_t *)s, len, ">>>");
    _bytes_out += len;
}

// An AT command: DLC 1 when multiplexed, straight at the UART otherwise.
void AP_LTE::send_raw(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len == 0 || _txq == nullptr) {
        return;
    }
    if (cmux_enabled()) {
        cmux_send(CMUX_DLC_AT, data, len);
    } else {
        _txq->write(data, len);
    }
    if (option_enabled(OPT_LOGALL) || _step != Step::CONNECTED) {
        log_raw(data, len, ">>>");
    }
    _bytes_out += len;
}

// The socket stream, which rides DLC 2 when multiplexed.
void AP_LTE::data_send(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len == 0 || _txq == nullptr) {
        return;
    }
    if (cmux_enabled()) {
        cmux_send(CMUX_DLC_DATA, data, len);
    } else {
        _txq->write(data, len);
    }
    if (option_enabled(OPT_LOGALL)) {
        log_raw(data, len, ">>>");
    }
    _bytes_out += len;
}

void AP_LTE::at_send(const char *cmd)
{
    if (cmd == nullptr) {
        return;
    }
    send_raw((const uint8_t *)cmd, uint16_t(strlen(cmd)));
}

/*
  Drain whatever the virtual port could not take last pass. While anything is
  held here the modem UART is left unread, so the backpressure reaches the
  modem instead of this driver dropping telemetry on the floor.
 */
void AP_LTE::flush_fc()
{
    if (_fc_len == 0) {
        return;
    }
    const size_t n = _port.device_write(_fc, _fc_len);
    if (n == 0) {
        return;
    }
    if (n >= _fc_len) {
        _fc_len = 0;
        return;
    }
    memmove(_fc, &_fc[n], _fc_len - n);
    _fc_len -= uint16_t(n);
}

void AP_LTE::at_sendf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    at_send(buf);
}

/*
  Format and send a command on the socket channel (DLC 2 when multiplexed).

  Quectel's transparent access mode binds the socket to whichever channel
  issued the open, so every command on the data path -- socket open, CIPMODE,
  NETOPEN, the PPP dial -- has to go out on DLC 2 and not the AT channel.
  Issued on DLC 1 the socket lands on the AT channel instead: the downlink is
  parsed as AT text and discarded, and every uplink byte is framed to a
  channel with no socket on it.

  Unlike bulk socket traffic these are part of the AT dialogue, so they are
  always logged rather than only under LOGALL.
*/
void AP_LTE::data_sendf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    const uint16_t len = uint16_t(strlen(buf));
    if (!option_enabled(OPT_LOGALL)) {
        log_raw((const uint8_t *)buf, len, ">>>");
    }
    data_send((const uint8_t *)buf, len);
}

/*
  Accumulate both mux channels into the reply buffer.

  A command sent on DLC 2 is answered there, but modems also echo socket
  status on the AT channel, so the steps that wait on the data path scan both.
  Unmuxed, _dlc2_len is always zero and this is just rx_append.
*/
void AP_LTE::rx_append_all()
{
    rx_append(_chunk, _chunk_len);
    rx_append(_dlc2, _dlc2_len);
}

void AP_LTE::uart_write_pending()
{
    if (_txq == nullptr || _uart == nullptr) {
        return;
    }
    uint32_t avail = _txq->available();
    while (avail > 0) {
        uint8_t tmp[128];
        const uint32_t want = MIN(avail, uint32_t(sizeof(tmp)));
        const uint32_t space = _uart->txspace();
        if (space == 0) {
            break;
        }
        const uint32_t n = MIN(want, space);
        _txq->peekbytes(tmp, n);
        const size_t written = _uart->write(tmp, n);
        if (written == 0) {
            break;
        }
        _txq->advance(written);
        avail -= written;
    }
}

void AP_LTE::send_data_reset()
{
    if (_modem->reset == nullptr) {
        return;
    }
    at_send(_modem->reset);
    if (!_modem->reset_not_baudrate) {
        _uart->begin(pget(_ibaud));
    }
    // After the reset command is queued, not before: at_send still has to
    // frame it if the modem is currently multiplexed.
    _found_cmux = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: sent reset");
}

bool AP_LTE::handle_error()
{
    if (rx_find("\nERROR\r\n")) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: error response from modem");
        send_data_reset();
        _step = Step::ATI;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// reply accumulator
//
// Binary safe throughout: in CONNECTED the same buffer carries raw socket
// bytes, which contain NULs, so nothing here may rely on NUL termination to
// find the end of the data. The trailing NUL exists only so the numeric
// parsers can sscanf() from a located token.
// ---------------------------------------------------------------------------

void AP_LTE::rx_reset()
{
    _rx_len = 0;
    _rx[0] = 0;
}

void AP_LTE::rx_append(const uint8_t *data, uint16_t len)
{
    if (len == 0) {
        return;
    }
    if (_rx_len + len > AP_LTE_RXBUF_SIZE) {
        // keep the tail: a reply we are still waiting for is at the end
        const uint16_t keep = AP_LTE_RXBUF_SIZE / 2;
        if (_rx_len > keep) {
            memmove(_rx, &_rx[_rx_len - keep], keep);
            _rx_len = keep;
        }
        if (_rx_len + len > AP_LTE_RXBUF_SIZE) {
            len = AP_LTE_RXBUF_SIZE - _rx_len;
        }
    }
    memcpy(&_rx[_rx_len], data, len);
    _rx_len += len;
    _rx[_rx_len] = 0;
}

const char *AP_LTE::mem_find(const char *hay, uint16_t hay_len, const char *needle)
{
    const uint16_t nlen = uint16_t(strlen(needle));
    if (nlen == 0 || hay_len < nlen) {
        return nullptr;
    }
    for (uint16_t i = 0; i + nlen <= hay_len; i++) {
        if (memcmp(&hay[i], needle, nlen) == 0) {
            return &hay[i];
        }
    }
    return nullptr;
}

const char *AP_LTE::rx_at(const char *needle) const
{
    return mem_find(_rx, _rx_len, needle);
}

bool AP_LTE::rx_find(const char *needle) const
{
    return rx_at(needle) != nullptr;
}

bool AP_LTE::chunk_find(const char *needle) const
{
    return mem_find((const char *)_chunk, _chunk_len, needle) != nullptr;
}

void AP_LTE::rx_consume_through(const char *needle)
{
    const char *p = rx_at(needle);
    if (p == nullptr) {
        return;
    }
    const uint16_t drop = uint16_t(p - _rx) + uint16_t(strlen(needle));
    if (drop >= _rx_len) {
        rx_reset();
        return;
    }
    memmove(_rx, &_rx[drop], _rx_len - drop);
    _rx_len -= drop;
    _rx[_rx_len] = 0;
}

// ---------------------------------------------------------------------------
// resets
// ---------------------------------------------------------------------------

void AP_LTE::reset_buffers()
{
    _last_data_ms = AP_HAL::millis();
    rx_reset();
    _fc_len = 0;
    _chunk_len = 0;
    _dlc2_len = 0;
    _cmux_partial_len = 0;
    _port.device_clear();
}

void AP_LTE::reset_state()
{
    _step = Step::ATI;
    _modem = &AP_LTE_default_modem;
    // _cmux_force_disabled deliberately survives: it records that this
    // modem's firmware cannot do CMUX, which a reset does not change.
    _found_cmux = false;
    _cmux_was_set = false;
    reset_buffers();
    if (_txq != nullptr) {
        _txq->clear();
    }
    _track_band = -1;
    _track_cid = -1;
    _cops_zero_sent = false;
    _qcsq_tries = 0;
    _cipopen_retry = 0;
    _cipopen_sent = false;
    _cipopen_sent_ms = 0;
    _hard_reset_strikes = 0;
    _sim_probe_count = 0;
    _sim_missing_count = 0;
    _cpin_probe_n = 0;
    _cereg_drop_ms = 0;
    _creg_search_ms = 0;
    _step_times_n = 0;
    _step_timer_ms = AP_HAL::millis();
    _post_reset = true;
    _cipopen_preclosed = false;
    _cipmode_sent = false;
    _ati_dbg_ms = 0;
    // auth_done/auth_ip/auth_port deliberately survive a modem hard reset.
    // Re-authenticating is wanted only on a fresh boot, not every time the
    // link drops. _http_sub still needs clearing so a half-finished attempt
    // restarts cleanly if a reset interrupted one.
    _http_sub = HttpSub::NONE;
    _http_retry_count = 0;
}

void AP_LTE::reset_to_ATI()
{
    const char *timed_out_step = step_name(_step);
    const uint32_t elapsed_ms = AP_HAL::millis() - _step_timer_ms;
    send_data_reset();
    uart_write_pending();
    reset_state();
    if (_step_times_n < ARRAY_SIZE(_step_times)) {
        _step_times[_step_times_n].name = timed_out_step;
        _step_times[_step_times_n].ms = elapsed_ms;
        _step_times_n++;
    }
    // tells the next step-changed check not to log this same transition twice
    _reset_recorded = true;
}

// ---------------------------------------------------------------------------
// operator / band selection
// ---------------------------------------------------------------------------

void AP_LTE::set_MCCMNC()
{
    if (_modem->mccmnc == nullptr) {
        return;
    }
    const int32_t mccmnc = pget(_mccmnc);
    if (mccmnc > 0) {
        at_sendf(_modem->mccmnc, unsigned(mccmnc));
    } else if (mccmnc == 0) {
        at_send("AT+COPS=0\r\n");
    }
    _last_mccmnc = mccmnc;
}

void AP_LTE::set_BAND()
{
    if (_modem->setband == nullptr && _modem->setband_mask == nullptr) {
        return;
    }
    const int32_t band = pget(_band);
    if (band > 0) {
        if (_modem->setband_mask != nullptr) {
            at_sendf(_modem->setband_mask, (unsigned long long)(1ULL << (band - 1)));
        } else {
            at_sendf(_modem->setband, int(band));
        }
    } else if (band == 0 && _modem->setband_all != nullptr) {
        at_send(_modem->setband_all);
    }
    _last_band = band;
}

void AP_LTE::effective_server(uint8_t ip[4], uint16_t &port) const
{
    if (_auth_port != 0) {
        memcpy(ip, _auth_ip, 4);
        port = _auth_port;
        return;
    }
    ip[0] = uint8_t(pget(_server_ip0));
    ip[1] = uint8_t(pget(_server_ip1));
    ip[2] = uint8_t(pget(_server_ip2));
    ip[3] = uint8_t(pget(_server_ip3));
    port = uint16_t(pget(_server_port));
}

// ---------------------------------------------------------------------------
// raw AT traffic log
// ---------------------------------------------------------------------------

void AP_LTE::log_open()
{
    uint32_t idx = 0;
    int fd = AP::FS().open("LTE_log_idx.txt", O_RDONLY);
    if (fd != -1) {
        char buf[16] {};
        const ssize_t n = AP::FS().read(fd, buf, sizeof(buf) - 1);
        AP::FS().close(fd);
        if (n > 0) {
            idx = uint32_t(strtoul(buf, nullptr, 10));
        }
    }
    idx++;

    fd = AP::FS().open("LTE_log_idx.txt", O_WRONLY | O_CREAT | O_TRUNC);
    if (fd != -1) {
        char buf[16];
        const int n = hal.util->snprintf(buf, sizeof(buf), "%lu", (unsigned long)idx);
        AP::FS().write(fd, buf, n);
        AP::FS().close(fd);
    }

    char name[32];
    hal.util->snprintf(name, sizeof(name), "LTE_modem_%04lu.log", (unsigned long)idx);
    _log_fd = AP::FS().open(name, O_WRONLY | O_CREAT | O_TRUNC);
}

/*
  One line per read or write, prefixed with the millisecond timestamp. The
  applet logged the same traffic without timestamps, which made ordering
  questions about intermittent HTTPAUTH failures unanswerable after the fact.
 */
void AP_LTE::log_raw(const uint8_t *data, uint16_t len, const char *marker)
{
    if (_log_fd == -1 || len == 0) {
        return;
    }
    char hdr[24];
    const int hn = hal.util->snprintf(hdr, sizeof(hdr), "%lu %s[",
                                      (unsigned long)AP_HAL::millis(), marker);
    AP::FS().write(_log_fd, hdr, hn);
    AP::FS().write(_log_fd, data, len);
    AP::FS().write(_log_fd, "]\n", 2);
    AP::FS().fsync(_log_fd);
}

namespace AP {
AP_LTE *lte()
{
    return AP_LTE::get_singleton();
}
}

#endif  // AP_LTE_ENABLED
