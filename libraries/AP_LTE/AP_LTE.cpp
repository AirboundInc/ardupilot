#include "AP_LTE.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_LTE::var_info[] = {
    AP_GROUPINFO("ENABLE", 1, AP_LTE, _enabled, 0),
    AP_GROUPINFO("DEBUG", 2, AP_LTE, _debug_level, 1),
    AP_GROUPEND
};

AP_LTE::AP_LTE()
    : _uart_modem(nullptr),
      _uart_mavlink(nullptr),
      _state(State::INIT),
      _state_start_ms(0),
      _connected(false),
      _rx_idx(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_LTE::init()
{
    if (!_enabled) {
        return;
    }
    if (_initialized)
        return;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: starting");

    _uart_modem = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_Scripting, 0);

    if (!_uart_modem) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: no modem UART");
        _state = State::ERROR;
        return;
    }

    _uart_modem->begin(115200);

    // _uart_mavlink = AP::serialmanager().find_serial(
    //     AP_SerialManager::SerialProtocol_MAVLink2, 1);

    // if (!_uart_mavlink) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: no MAVLink UART");
    //     _state = State::ERROR;
    //     return;
    // }

    change_state(State::WAIT_BOOT);
    _initialized=true;
}

void AP_LTE::update()
{
    if (!_enabled || _state == State::ERROR) {
        return;
    }

    if (_state == State::CONNECTED) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: Bridging data");
        bridge_data();
    } else {
        handle_state_machine();
    }
}

void AP_LTE::handle_state_machine()
{
    uint32_t now = AP_HAL::millis();
    uint32_t elapsed = now - _state_start_ms;

    while (_uart_modem->available() && _rx_idx < sizeof(_rx_buf) - 1) {
        _rx_buf[_rx_idx++] = _uart_modem->read();
        _rx_buf[_rx_idx] = 0;
    }

    switch (_state)
    {

    case State::WAIT_BOOT:

        if (strstr(_rx_buf, "CPIN: READY") ||
            strstr(_rx_buf, "CFUN: 1")) {

            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: found modem");
            change_state(State::SEND_AT);
        }

        if (elapsed > 8000) {
            change_state(State::SEND_AT);
        }
        break;

    case State::SEND_AT:
        send_at("AT\r\n");
        change_state(State::WAIT_AT_OK);
        break;

    case State::WAIT_AT_OK:

        if (check_response("OK")) {
            change_state(State::CHECK_SIM);
        } 
        else if (elapsed > 3000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE_modem: timeout");
            change_state(State::SEND_AT);
        }
        break;

    case State::CHECK_SIM:
        send_at("AT+CPIN?\r\n");
        change_state(State::WAIT_SIM_OK);
        break;

    case State::WAIT_SIM_OK:

        if (check_response("READY")) {
            change_state(State::CHECK_NETWORK);
        } 
        else if (elapsed > 5000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE_modem: timeout");
            change_state(State::CHECK_SIM);
        }
        break;

    case State::CHECK_NETWORK:
        send_at("AT+CEREG?\r\n");
        change_state(State::WAIT_NETWORK);
        break;

    case State::WAIT_NETWORK:

        if (strstr(_rx_buf, "+CEREG: 0,1") ||
            strstr(_rx_buf, "+CEREG: 0,5")) {

            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: CREG OK");
            change_state(State::OPEN_SOCKET);
        }
        else if (elapsed > 10000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE_modem: timeout");
            change_state(State::CHECK_NETWORK);
        }
        break;

    case State::OPEN_SOCKET:

        send_at("AT+QIOPEN=1,0,\"UDP\",\"15.207.104.210\",16550,6001,2\r\n");
        change_state(State::WAIT_SOCKET);
        break;

    case State::WAIT_SOCKET:

        if (check_response("+QIOPEN: 0,0")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: network opened");
            change_state(State::SET_TRANSPARENT);
        }
        else if (strstr(_rx_buf, "+QIOPEN: 0,")) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: error response from modem");
            change_state(State::OPEN_SOCKET);
        }
        else if (check_response("CONNECT")){
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: network opened 2");
            change_state(State::CONNECTED);
            _connected = true;
        }
        break;

    case State::SET_TRANSPARENT:

        send_at("AT+QISWTMD=0,1\r\n");
        change_state(State::WAIT_TRANSPARENT);
        break;

    case State::WAIT_TRANSPARENT:

        if (check_response("CONNECT")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: transparent mode set");
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: connected");
            _connected = true;
            change_state(State::CONNECTED);
        }
        break;

    case State::CONNECTED:
        break;

    case State::ERROR:
    case State::INIT:
        break;
    }
}

void AP_LTE::bridge_data()
{
    if (!_connected) return;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: entered bridge");
    // if (_uart_mavlink->available()) {
    //     // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: entered bridge");
    //     uint8_t buf[256];
    //     int16_t n = _uart_mavlink->read(buf, sizeof(buf));
    //     if (n > 0) {
    //         _uart_modem->write(buf, n);
    //     }
    // }
    // else
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: no mavlink data");
 

    if (_uart_modem->available()) {
        uint8_t buf[256];
        int16_t n = _uart_modem->read(buf, sizeof(buf));
        if (n > 0) {

            if (memmem(buf, n, "QIURC: \"closed\"", 15)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "LTE_modem: connection closed, reconnecting");

                _connected = false;
                change_state(State::OPEN_SOCKET);
                return;
            }

            // _uart_mavlink->write(buf, n);
        }
    }
    else
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: no modem data");

}

void AP_LTE::send_at(const char *cmd)
{
    _rx_idx = 0;
    _uart_modem->write((const uint8_t*)cmd, strlen(cmd));
}

bool AP_LTE::check_response(const char *expected)
{
    if (_rx_idx == 0) return false;

    if (strstr(_rx_buf, expected)) {
        _rx_idx = 0;
        return true;
    }

    return false;
}

void AP_LTE::change_state(State new_state)
{
    _state = new_state;
    _state_start_ms = AP_HAL::millis();
    _rx_idx = 0;
}
