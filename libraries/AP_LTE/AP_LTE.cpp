#include "AP_LTE.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>
#include <inttypes.h>
#include <new>
#include <string.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// Server config — must match mavrelay.py
#define LTE_SERVER_IP    "15.207.104.210"
#define LTE_SERVER_PORT   16550
#define LTE_LOCAL_PORT    6001

#define LTE_VPORT_TXSIZE  4096
#define LTE_VPORT_RXSIZE  2048

// ─────────────────────────────────────────────────────────────────────────────
const AP_Param::GroupInfo AP_LTE::var_info[] = {
    AP_GROUPINFO("ENABLE", 1, AP_LTE, _enabled,     0),
    AP_GROUPINFO("DEBUG",  2, AP_LTE, _debug_level, 1),
    AP_GROUPEND
};

static const char *state_name(AP_LTE::State s)
{
    switch (s) {
    case AP_LTE::State::INIT:           return "INIT";
    case AP_LTE::State::WAIT_BOOT:      return "WAIT_BOOT";
    case AP_LTE::State::SEND_AT:        return "SEND_AT";
    case AP_LTE::State::WAIT_AT_OK:     return "WAIT_AT_OK";
    case AP_LTE::State::DISABLE_ECHO:   return "DISABLE_ECHO";
    case AP_LTE::State::WAIT_ECHO:      return "WAIT_ECHO";
    case AP_LTE::State::CHECK_SIM:      return "CHECK_SIM";
    case AP_LTE::State::WAIT_SIM_OK:    return "WAIT_SIM_OK";
    case AP_LTE::State::CHECK_NETWORK:  return "CHECK_NETWORK";
    case AP_LTE::State::WAIT_NETWORK:   return "WAIT_NETWORK";
    case AP_LTE::State::CLOSE_SOCKET:   return "CLOSE_SOCKET";
    case AP_LTE::State::WAIT_CLOSE:     return "WAIT_CLOSE";
    case AP_LTE::State::OPEN_SOCKET:    return "OPEN_SOCKET";
    case AP_LTE::State::WAIT_SOCKET:    return "WAIT_SOCKET";
    case AP_LTE::State::CONNECTED:      return "CONNECTED";
    case AP_LTE::State::ERROR:          return "ERROR";
    default:                            return "?";
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// VirtualPort
// ═════════════════════════════════════════════════════════════════════════════

bool AP_LTE_VirtualPort::init_buffers(uint32_t rxS, uint32_t txS)
{
    if (txS == last_size_tx && rxS == last_size_rx) return true;
    WITH_SEMAPHORE(sem);
    if (!writebuffer) writebuffer = new (std::nothrow) ByteBuffer(txS);
    else              writebuffer->set_size_best(txS);
    if (!readbuffer)  readbuffer  = new (std::nothrow) ByteBuffer(rxS);
    else              readbuffer->set_size_best(rxS);
    last_size_tx = txS; last_size_rx = rxS;
    return (writebuffer && readbuffer);
}

void AP_LTE_VirtualPort::_begin(uint32_t, uint16_t rxS, uint16_t txS)
{
    init_buffers(MAX((uint32_t)rxS, (uint32_t)LTE_VPORT_RXSIZE),
                 MAX((uint32_t)txS, (uint32_t)LTE_VPORT_TXSIZE));
}

size_t   AP_LTE_VirtualPort::_write(const uint8_t *b, size_t s)
{ WITH_SEMAPHORE(sem); return writebuffer ? writebuffer->write(b, s) : 0; }

ssize_t  AP_LTE_VirtualPort::_read(uint8_t *b, uint16_t c)
{ WITH_SEMAPHORE(sem); return readbuffer ? readbuffer->read(b, c) : -1; }

uint32_t AP_LTE_VirtualPort::_available()
{ WITH_SEMAPHORE(sem); return readbuffer ? readbuffer->available() : 0; }

uint32_t AP_LTE_VirtualPort::txspace()
{ WITH_SEMAPHORE(sem); return writebuffer ? writebuffer->space() : 0; }

bool AP_LTE_VirtualPort::_discard_input()
{ WITH_SEMAPHORE(sem); if (readbuffer) readbuffer->clear(); return true; }

uint32_t AP_LTE_VirtualPort::vport_available()
{ WITH_SEMAPHORE(sem); return writebuffer ? writebuffer->available() : 0; }

ssize_t  AP_LTE_VirtualPort::vport_read(uint8_t *buf, uint16_t count)
{ WITH_SEMAPHORE(sem); return writebuffer ? writebuffer->read(buf, count) : -1; }

size_t   AP_LTE_VirtualPort::vport_write(const uint8_t *buf, size_t size)
{ WITH_SEMAPHORE(sem); return readbuffer ? readbuffer->write(buf, size) : 0; }

// ═════════════════════════════════════════════════════════════════════════════
// AP_LTE
// ═════════════════════════════════════════════════════════════════════════════

AP_LTE::AP_LTE()
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_rx_buf, 0, sizeof(_rx_buf));
    memset(_tx_buf, 0, sizeof(_tx_buf));
}

// ─────────────────────────────────────────────────────────────────────────────
// pre_init / init / update
// ─────────────────────────────────────────────────────────────────────────────

void AP_LTE::pre_init()
{
    if (!_enabled) return;
    _vport.state.idx = AP_SERIALMANAGER_NET_PORT_1 + 1;
    _vport.state.protocol.set_and_save_ifchanged(
        (int8_t)AP_SerialManager::SerialProtocol_MAVLink2);
    _vport.begin(1000000, LTE_VPORT_RXSIZE, LTE_VPORT_TXSIZE);
    AP::serialmanager().register_port(&_vport);
    _vport_registered = true;
}

void AP_LTE::init()
{
    if (!_enabled || _initialized) return;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: starting");

    _uart_modem = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_Scripting, 0);
    if (!_uart_modem) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "LTE_modem: no modem UART (need SERIAL1_PROTOCOL=28)");
        _state = State::ERROR;
        return;
    }
    _uart_modem->begin(115200);

    // Verify vport was picked up by GCS
    bool found = false;
    for (uint8_t i = 0; i < 4; i++) {
        AP_HAL::UARTDriver *p = AP::serialmanager().find_serial(
            AP_SerialManager::SerialProtocol_MAVLink2, i);
        if (p == nullptr) break;
        if (p == &_vport) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: vport=GCS_MAVLink2[%u] OK", i);
            found = true;
            break;
        }
    }
    if (!found) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE_modem: vport NOT found by GCS");
    }

    change_state(State::WAIT_BOOT);
    _initialized = true;
}

void AP_LTE::update()
{
    if (!_enabled) return;

    if (!_initialized) {
        if (AP_HAL::millis() > 5000) {
            init();
        }
        return;
    }

    if (_state == State::ERROR) return;

    if (_state == State::CONNECTED) {
        bridge_data();
    } else {
        handle_state_machine();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State machine
// ─────────────────────────────────────────────────────────────────────────────

void AP_LTE::handle_state_machine()
{
    uint32_t now     = AP_HAL::millis();
    uint32_t elapsed = now - _state_start_ms;

    while (_uart_modem->available() && _rx_idx < RX_BUF_SIZE - 1) {
        _rx_buf[_rx_idx++] = (uint8_t)_uart_modem->read();
    }

    switch (_state)
    {
    case State::WAIT_BOOT:
        if (strstr((char*)_rx_buf, "PB DONE")     ||
            strstr((char*)_rx_buf, "SMS DONE")    ||
            strstr((char*)_rx_buf, "CFUN: 1")     ||
            strstr((char*)_rx_buf, "CPIN: READY") ||
            elapsed > 8000) {
            change_state(State::SEND_AT);
        }
        break;

    case State::SEND_AT:
        send_at("AT\r\n");
        change_state(State::WAIT_AT_OK);
        break;

    case State::WAIT_AT_OK:
        if (elapsed < 100) break;
        if (check_response("OK")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: AT OK rxd");
            change_state(State::DISABLE_ECHO);
            break;
        }
        if (elapsed > 3000) {
            char hex[32] = {};
            uint8_t n = MIN(_rx_idx, (uint8_t)8);
            for (uint8_t i = 0; i < n; i++) {
                hal.util->snprintf(hex + i*3, 4, "%02X ", _rx_buf[i]);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: AT timeout rx=%u [%s]", _rx_idx, hex);
            change_state(State::SEND_AT);
        }
        break;

    case State::DISABLE_ECHO:
        // ATE0      = disable echo
        // ATS2=255  = disable +++ escape sequence (prevents modem entering AT mode)
        // AT+QIEXTSEND=0 = disable extended send mode
        _uart_modem->write((const uint8_t*)"ATE0\r\n", 7);
        _rx_idx = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
        change_state(State::WAIT_ECHO);
        break;

    case State::WAIT_ECHO:
        if (elapsed < 500) break;
        if (check_response("OK") || elapsed > 3000) {
            // Disable +++ escape to prevent MAVLink bytes triggering AT mode
            _uart_modem->write((const uint8_t*)"ATS2=255\r\n", 11);
            _rx_idx = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            change_state(State::CHECK_SIM);
        }
        break;

    case State::CHECK_SIM:
        send_at("AT+CPIN?\r\n");
        change_state(State::WAIT_SIM_OK);
        break;

    case State::WAIT_SIM_OK:
        if (elapsed < 200) break;
        if (check_response("READY")) { change_state(State::CHECK_NETWORK); break; }
        if (elapsed > 10000)         { change_state(State::CHECK_SIM); }
        break;

    case State::CHECK_NETWORK:
        send_at("AT+CREG?\r\n");
        change_state(State::WAIT_NETWORK);
        break;

    case State::WAIT_NETWORK:
        if (elapsed < 300) break;
        if (strstr((char*)_rx_buf, "+CREG: 0,1") ||
            strstr((char*)_rx_buf, "+CREG: 0,5") ||
            strstr((char*)_rx_buf, "+CREG: 1")   ||
            strstr((char*)_rx_buf, "+CREG: 5")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: CREG OK");
            change_state(State::CLOSE_SOCKET);
        } else if (elapsed > 2000) {
            change_state(State::CHECK_NETWORK);
        }
        break;

    case State::CLOSE_SOCKET:
        _uart_modem->write((const uint8_t*)"AT+QICLOSE=0\r\n", 14);
        change_state(State::WAIT_CLOSE);
        break;

    case State::WAIT_CLOSE:
        if (elapsed < 300) break;
        if (elapsed > 3000 ||
            strstr((char*)_rx_buf, "OK") ||
            strstr((char*)_rx_buf, "ERROR")) {
            change_state(State::OPEN_SOCKET);
        }
        break;

    case State::OPEN_SOCKET:
        {
            char cmd[96];
            hal.util->snprintf(cmd, sizeof(cmd),
                               "AT+QIOPEN=1,0,\"UDP\",\"%s\",%u,%u,2\r\n",
                               LTE_SERVER_IP,
                               (unsigned)LTE_SERVER_PORT,
                               (unsigned)LTE_LOCAL_PORT);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: opening UDP %s:%u",
                          LTE_SERVER_IP, (unsigned)LTE_SERVER_PORT);
            _uart_modem->write((const uint8_t*)cmd, strlen(cmd));
        }
        change_state(State::WAIT_SOCKET);
        break;

    case State::WAIT_SOCKET:
        if (elapsed < 200) break;
        {
            static uint32_t _last_rxdump_ms = 0;
            if (now - _last_rxdump_ms > 3000) {
                _last_rxdump_ms = now;
                char hex[64] = {};
                uint16_t n = MIN(_rx_idx, (uint16_t)8);
                for (uint16_t i = 0; i < n; i++) {
                    hal.util->snprintf(hex + i*3, 4, "%02X ", _rx_buf[i]);
                }
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "LTE_modem: rxbuf[%u]: %s", _rx_idx, hex);
            }
        }
        if (strstr((char*)_rx_buf, "+QIOPEN: 0,0") ||
            strstr((char*)_rx_buf, "CONNECT")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: connected");
            _connected = true;
            _rx_idx = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            change_state(State::CONNECTED);
            return;
        }
        if (strstr((char*)_rx_buf, "ERROR")) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE_modem: QIOPEN ERROR, retrying");
            change_state(State::CLOSE_SOCKET);
        }
        if (elapsed > 30000) {
            change_state(State::CLOSE_SOCKET);
        }
        break;

    case State::CONNECTED:
    case State::ERROR:
    case State::INIT:
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// bridge_data  (access mode 2 — direct push, same as Lua 4.6.3)
//   Uplink:   vport → AT+QISEND → modem → relay → MP
//   Downlink: modem pushes data directly to UART as raw bytes (no QIRD needed)
//             Unsolicited: +QIURC: "recv",0\r\n<len>\r\n<data>
// ─────────────────────────────────────────────────────────────────────────────
void AP_LTE::bridge_data()
{
    uint32_t now = AP_HAL::millis();

    // ── Drain modem UART into rx_buf ─────────────────────────────────────────
    while (_uart_modem->available() && _rx_idx < RX_BUF_SIZE - 1) {
        _rx_buf[_rx_idx++] = (uint8_t)_uart_modem->read();
    }
    _rx_buf[_rx_idx] = 0;

    // ── Detect modem reboot / connection closed ───────────────────────────────
    if (strstr((char*)_rx_buf, "PB DONE")) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE_modem: modem reboot -> reconnect");
        _connected = false;
        _tx_pending = false;
        _rx_idx = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
        change_state(State::WAIT_BOOT);
        return;
    }
    if (strstr((char*)_rx_buf, "+QIURC: \"closed\"") ||
        strstr((char*)_rx_buf, "\r\nCLOSED\r\n")) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: connection closed, reconnecting");
        _connected = false;
        _tx_pending = false;
        _rx_idx = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
        change_state(State::CLOSE_SOCKET);
        return;
    }

    // ── Uplink diagnostics every 3s ──────────────────────────────────────────
    {
        static uint32_t _last_diag_ms = 0;
        if (now - _last_diag_ms > 3000) {
            _last_diag_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: vport_avail=%u",
                          (unsigned)_vport.vport_available());
        }
    }

    // ── Downlink: transparent mode — raw bytes arrive directly on UART ─────────
    // No AT framing, no QIURC, no QIRD — just pass bytes straight to vport
    // Only skip bytes if we're mid-uplink (waiting for QISEND prompt)
    if (_rx_idx > 0) {
        // Log if looks like MAVLink
        if (_rx_buf[0] == 0xFD || _rx_buf[0] == 0xFE) {
            char hex[52] = {};
            uint8_t hn = (uint8_t)MIN((int)_rx_idx, 16);
            for (uint8_t i = 0; i < hn; i++) {
                hal.util->snprintf(hex + i*3, 4, "%02X ", _rx_buf[i]);
            }
            uint32_t msgid = 0;
            const char *mav_type = "RAW";
            if (_rx_idx >= 10 && _rx_buf[0] == 0xFD) {
                msgid = (uint32_t)_rx_buf[7] |
                        ((uint32_t)_rx_buf[8] << 8) |
                        ((uint32_t)_rx_buf[9] << 16);
                mav_type = "MAV2";
            } else if (_rx_idx >= 6 && _rx_buf[0] == 0xFE) {
                msgid = _rx_buf[5];
                mav_type = "MAV1";
            }
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "MP->PX %s msgid=%u len=%u: %s",
                          mav_type, (unsigned)msgid, (unsigned)_rx_idx, hex);
        }
        size_t written = _vport.vport_write(_rx_buf, _rx_idx);
        if (written != _rx_idx) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: vport partial %u/%u",
                          (unsigned)written, (unsigned)_rx_idx);
        }
        _rx_idx = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
    }

    // ── Uplink: read vport and write raw bytes directly to UART ──────────────
    // In transparent mode no AT+QISEND needed — just write raw bytes
    uint32_t avail = _vport.vport_available();
    if (avail > 0) {
        uint16_t n = (uint16_t)MIN(avail, (uint32_t)100U);
        ssize_t got = _vport.vport_read(_tx_buf, n);
        if (got > 0) {
            _uart_modem->write(_tx_buf, (uint16_t)got);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

void AP_LTE::send_at(const char *cmd)
{
    _rx_idx = 0;
    memset(_rx_buf, 0, sizeof(_rx_buf));
    _uart_modem->write((const uint8_t*)cmd, strlen(cmd));
    if (_debug_level >= 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                      "LTE_modem: [%s] >> %.50s", state_name(_state), cmd);
    }
}

bool AP_LTE::check_response(const char *expected)
{
    if (!_rx_idx) return false;
    if (strstr((char*)_rx_buf, expected)) {
        if (_debug_level >= 1) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                          "LTE_modem: [%s] matched \"%s\"",
                          state_name(_state), expected);
        }
        _rx_idx = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
        return true;
    }
    return false;
}

void AP_LTE::change_state(State new_state)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "LTE_modem: %s -> %s",
                  state_name(_state), state_name(new_state));
    _state          = new_state;
    _state_start_ms = AP_HAL::millis();
    _rx_idx         = 0;
    memset(_rx_buf, 0, sizeof(_rx_buf));
}