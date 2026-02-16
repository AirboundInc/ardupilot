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

    _uart_modem = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_Scripting, 0);

    if (!_uart_modem) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE: No modem UART");
        _state = State::ERROR;
        return;
    }

    _uart_modem->begin(115200);

    _uart_mavlink = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_MAVLink2, 0);

    if (!_uart_mavlink) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE: No MAVLink UART");
        _state = State::ERROR;
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Init OK");
    change_state(State::WAIT_BOOT);
    _initialized=true;
}

void AP_LTE::update()
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE update running");
    if (!_enabled || _state == State::ERROR) {
        return;
    }

    if (_state == State::CONNECTED) {
        bridge_data();
    } else {
        handle_state_machine();
    }
}

void AP_LTE::handle_state_machine()
{
    uint32_t now = AP_HAL::millis();
    uint32_t elapsed = now - _state_start_ms;

    // Read modem
    while (_uart_modem->available() && _rx_idx < sizeof(_rx_buf)-1) {
        _rx_buf[_rx_idx++] = _uart_modem->read();
        _rx_buf[_rx_idx] = 0;
    }

    switch (_state)
    {
        case State::SEND_AT:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Sending AT");
            send_at("AT\r\n");
            change_state(State::WAIT_AT_OK);    
            break;

        case State::WAIT_AT_OK:

            // Print whatever we received from modem
            if (_rx_idx > 0) {
                char temp[100];
                uint16_t copy_len = MIN(_rx_idx, (uint16_t)(sizeof(temp) - 1));
                memcpy(temp, _rx_buf, copy_len);
                temp[copy_len] = '\0';

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE RX: %s", temp);
            }

            if (check_response("OK")) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: AT OK received");
                change_state(State::CHECK_SIM);
            } 
            else if (elapsed > 3000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: AT timeout, retrying");
                change_state(State::WAIT_BOOT);
            }

            break;

        case State::CHECK_SIM:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Checking SIM");
            send_at("AT+CPIN?\r\n");
            change_state(State::WAIT_SIM_OK);
            break;


        case State::WAIT_SIM_OK:
            // if (check_response("READY")) {
            //     change_state(State::CHECK_NETWORK);
            // } else if (elapsed > 5000) {
            //     change_state(State::CHECK_SIM);
            // }
            // break;
            if (_rx_idx > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE RX: %s", _rx_buf);
            }

            if (check_response("READY")) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: SIM READY");
                change_state(State::CHECK_NETWORK);
            } 
            else if (elapsed > 5000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: SIM timeout");
                change_state(State::CHECK_SIM);
            }

            break;

        case State::CHECK_NETWORK:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Checking Network");
            send_at("AT+CEREG?\r\n");
            change_state(State::WAIT_NETWORK);
            break;

        case State::WAIT_NETWORK:

            if (_rx_idx > 0) {
                char temp[120];
                uint16_t copy_len = MIN(_rx_idx, (uint16_t)(sizeof(temp) - 1));
                memcpy(temp, _rx_buf, copy_len);
                temp[copy_len] = '\0';

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE RX: %s", temp);
            }

            if (strstr(_rx_buf, "+CEREG: 0,1") ||
                strstr(_rx_buf, "+CEREG: 0,5")) {

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Network Registered!");
                change_state(State::SETUP_APN);
            }
            else if (elapsed > 10000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: Network timeout, retrying");
                change_state(State::CHECK_NETWORK);
            }

            break;


        case State::SETUP_APN:
            send_at("AT+QICSGP=1,1,\"cmnet\",\"\",\"\",0\r\n");
            change_state(State::WAIT_APN_OK);
            break;

        case State::WAIT_APN_OK:
            if (check_response("OK")) {
                change_state(State::ACTIVATE_CONTEXT);
            } else if (elapsed > 5000) {
                change_state(State::SETUP_APN);
            }
            break;

        case State::ACTIVATE_CONTEXT:
            send_at("AT+QIACT=1\r\n");
            change_state(State::WAIT_CONTEXT_OK);
            break;

        case State::WAIT_CONTEXT_OK:
            if (check_response("OK")) {
                change_state(State::OPEN_UDP);
            } else if (elapsed > 10000) {
                change_state(State::ACTIVATE_CONTEXT);
            }
            break;

        case State::OPEN_UDP:
            send_at("AT+QIOPEN=1,0,\"UDP\",\"13.203.94.240\",16550,6001,0\r\n");
            change_state(State::WAIT_UDP_OK);
            break;

        case State::WAIT_UDP_OK:
            if (check_response("+QIOPEN: 0,0")) {
                change_state(State::ENABLE_TRANSPARENT);
            } else if (elapsed > 15000) {
                change_state(State::OPEN_UDP);
            }
            break;

        case State::ENABLE_TRANSPARENT:
            send_at("AT+QISWTMD=0,1\r\n");
            change_state(State::WAIT_TRANSPARENT_OK);
            break;

        case State::WAIT_TRANSPARENT_OK:
            if (check_response("OK")) {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "LTE: CONNECTED");
                _connected = true;
                change_state(State::CONNECTED);
            } else if (elapsed > 5000) {
                change_state(State::ENABLE_TRANSPARENT);
            }
            break;

        case State::CONNECTED:
            break;

        case State::INIT:
        case State::ERROR:
            break;

        case State::WAIT_BOOT:

            if (_rx_idx > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE RX: %s", _rx_buf);
            }

            // If modem indicates SIM ready or full function
            if (strstr(_rx_buf, "CPIN: READY") ||
                strstr(_rx_buf, "CFUN: 1")) {

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Modem boot complete");
                change_state(State::SEND_AT);
            }

            // Safety fallback: after 8 seconds try AT anyway
            if (elapsed > 8000) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Boot wait timeout, sending AT");
                change_state(State::SEND_AT);
            }

            break;

    }
}

void AP_LTE::bridge_data()
{
    if (!_connected) return;

    // MAVLink → LTE
    if (_uart_mavlink->available()) {
        uint8_t buf[256];
        int16_t n = _uart_mavlink->read(buf, sizeof(buf));
        if (n > 0) {
            _uart_modem->write(buf, n);
        }
    }

    // LTE → MAVLink
    if (_uart_modem->available()) {
        uint8_t buf[256];
        int16_t n = _uart_modem->read(buf, sizeof(buf));
        if (n > 0) {

            // detect disconnect
            if (memmem(buf, n, "QIURC: \"closed\"", 15)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: Disconnected");
                _connected = false;
                change_state(State::OPEN_UDP);
                return;
            }

            _uart_mavlink->write(buf, n);
        }
    }
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
