#include "AP_LTE.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <inttypes.h>

extern const AP_HAL::HAL& hal;

// ─────────────────────────────────────────────────────────────────────────────
// Server config — UDP
// ─────────────────────────────────────────────────────────────────────────────
#define LTE_SERVER_IP    "15.207.104.210"
#define LTE_SERVER_PORT   16550
#define LTE_LOCAL_PORT    16550

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
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
    case AP_LTE::State::ACTIVATE_PDP:   return "ACTIVATE_PDP";
    case AP_LTE::State::WAIT_PDP:       return "WAIT_PDP";
    case AP_LTE::State::WAIT_ACT:       return "WAIT_ACT";
    case AP_LTE::State::CHECK_ACT:      return "CHECK_ACT";
    case AP_LTE::State::WAIT_ACT_CHECK: return "WAIT_ACT_CHECK";
    case AP_LTE::State::CLOSE_SOCKET:   return "CLOSE_SOCKET";
    case AP_LTE::State::WAIT_CLOSE:     return "WAIT_CLOSE";
    case AP_LTE::State::OPEN_SOCKET:    return "OPEN_SOCKET";
    case AP_LTE::State::WAIT_SOCKET:    return "WAIT_SOCKET";
    case AP_LTE::State::CONNECTED:      return "CONNECTED";
    case AP_LTE::State::ERROR:          return "ERROR";
    default:                            return "UNKNOWN";
    }
}

AP_LTE::AP_LTE()
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_rx_buf,         0, sizeof(_rx_buf));
    memset(_pending_tx_buf, 0, sizeof(_pending_tx_buf));
}

void AP_LTE::init()
{
    if (!_enabled || _initialized) return;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: [INIT] starting");

    _uart_modem = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_Scripting, 0);
    if (!_uart_modem) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: [INIT] no modem UART");
        _state = State::ERROR;
        return;
    }
    _uart_modem->begin(115200);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: [INIT] modem UART @115200");

    // SERIAL2 used only for downlink (writing MP commands back to ArduPlane)
    _uart_telem = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_MAVLink2, 1);
    if (_uart_telem) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "LTE_modem: [INIT] telem UART found (SERIAL2)");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "LTE_modem: [INIT] no telem UART - set SERIAL2_PROTOCOL=2");
    }

    change_state(State::WAIT_BOOT);
    _initialized = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: [INIT] done -> WAIT_BOOT");
}

void AP_LTE::update()
{
    if (!_enabled || _state == State::ERROR) return;

    if (_state == State::CONNECTED) {
        if (_debug_level >= 1) {
            static uint32_t _last_update_log = 0;
            uint32_t n = AP_HAL::millis();
            if (n - _last_update_log > 3000) {
                _last_update_log = n;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "LTE_modem: update() CONNECTED, calling bridge");
            }
        }
        bridge_data();
    } else {
        handle_state_machine();
    }
}

void AP_LTE::handle_state_machine()
{
    uint32_t now     = AP_HAL::millis();
    uint32_t elapsed = now - _state_start_ms;

    while (_uart_modem->available() && _rx_idx < sizeof(_rx_buf) - 1) {
        _rx_buf[_rx_idx++] = (char)_uart_modem->read();
    }

    switch (_state)
    {
    case State::WAIT_BOOT:
        if (strstr(_rx_buf, "CPIN: READY") ||
            strstr(_rx_buf, "CFUN: 1")     ||
            strstr(_rx_buf, "PB DONE")     ||
            strstr(_rx_buf, "SMS DONE")) {
            change_state(State::SEND_AT);
            break;
        }
        if (elapsed > 8000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: [WAIT_BOOT] timeout -> SEND_AT");
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
            change_state(State::DISABLE_ECHO);
        } else if (elapsed > 3000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: [WAIT_AT_OK] timeout, retrying");
            change_state(State::SEND_AT);
        }
        break;

    case State::DISABLE_ECHO:
        send_at("ATE0\r\n");
        change_state(State::WAIT_ECHO);
        break;

    case State::WAIT_ECHO:
        if (elapsed < 200) break;
        if (check_response("OK") || elapsed > 2000) {
            change_state(State::CHECK_SIM);
        }
        break;

    case State::CHECK_SIM:
        send_at("AT+CPIN?\r\n");
        change_state(State::WAIT_SIM_OK);
        break;

    case State::WAIT_SIM_OK:
        if (elapsed < 200) break;
        if (check_response("READY")) {
            change_state(State::CHECK_NETWORK);
        } else if (check_response("SIM PIN") || check_response("SIM PUK")) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: SIM locked -> ERROR");
            _state = State::ERROR;
        } else if (elapsed > 5000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: [WAIT_SIM_OK] timeout, retrying");
            change_state(State::CHECK_SIM);
        }
        break;

    case State::CHECK_NETWORK:
        send_at("AT+CREG?\r\n");
        change_state(State::WAIT_NETWORK);
        break;

    case State::WAIT_NETWORK:
        if (elapsed < 200) break;
        if (strstr(_rx_buf, "+CREG: 0,1") ||
            strstr(_rx_buf, "+CREG: 0,5")) {
            change_state(State::ACTIVATE_PDP);
        } else if (elapsed > 10000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: [WAIT_NETWORK] timeout buf=\"%.40s\"",
                          _rx_buf);
            change_state(State::CHECK_NETWORK);
        }
        break;

    case State::ACTIVATE_PDP:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "LTE_modem: [ACTIVATE_PDP] deact context 1 first");
        send_at("AT+QIDEACT=1\r\n");
        change_state(State::WAIT_PDP);
        break;

    case State::WAIT_PDP:
        if (elapsed < 500) break;
        if (check_response("OK") || check_response("ERROR") || elapsed > 8000) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: [WAIT_PDP] deact done, activating");
            send_at("AT+QIACT=1\r\n");
            change_state(State::WAIT_ACT);
        }
        break;

    case State::WAIT_ACT:
        if (elapsed < 500) break;
        if (check_response("OK") || check_response("ERROR") || elapsed > 150000) {
            change_state(State::CHECK_ACT);
        }
        break;

    case State::CHECK_ACT:
        send_at("AT+QIACT?\r\n");
        change_state(State::WAIT_ACT_CHECK);
        break;

    case State::WAIT_ACT_CHECK:
        if (elapsed < 500) break;
        if (strstr(_rx_buf, "+QIACT: 1,1")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: [WAIT_ACT_CHECK] context 1 verified UP "
                          "-> CLOSE_SOCKET");
            change_state(State::CLOSE_SOCKET);
        } else if (elapsed > 5000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: [WAIT_ACT_CHECK] context NOT active "
                          "buf=\"%.60s\", retrying QIACT",
                          _rx_buf);
            change_state(State::ACTIVATE_PDP);
        }
        break;

    case State::CLOSE_SOCKET:
        send_at("AT+QICLOSE=0\r\n");
        change_state(State::WAIT_CLOSE);
        break;

    case State::WAIT_CLOSE:
        if (elapsed < 200) break;
        if (check_response("OK") || check_response("ERROR") || elapsed > 5000) {
            change_state(State::OPEN_SOCKET);
        }
        break;

    case State::OPEN_SOCKET:
        {
            char cmd[96];
            hal.util->snprintf(cmd, sizeof(cmd),
                               "AT+QIOPEN=1,0,\"UDP\",\"%s\",%u,%u\r\n",
                               LTE_SERVER_IP,
                               (unsigned)LTE_SERVER_PORT,
                               (unsigned)LTE_LOCAL_PORT);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: [OPEN_SOCKET] UDP %s:%u",
                          LTE_SERVER_IP, (unsigned)LTE_SERVER_PORT);
            send_at(cmd);
        }
        change_state(State::WAIT_SOCKET);
        break;

    case State::WAIT_SOCKET:
        if (elapsed < 200) break;
        if (strstr(_rx_buf, "+QIOPEN: 0,0")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LTE_modem: [WAIT_SOCKET] connected -> CONNECTED");
            _connected = true;
            change_state(State::CONNECTED);
        } else if (strstr(_rx_buf, "+QIOPEN: 0,")) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                          "LTE_modem: [WAIT_SOCKET] error buf=\"%.60s\"",
                          _rx_buf);
            change_state(State::CLOSE_SOCKET);
        } else if (elapsed > 30000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: [WAIT_SOCKET] timeout buf=\"%.80s\"",
                          _rx_buf);
            change_state(State::CLOSE_SOCKET);
        }
        break;

    case State::CONNECTED:
    case State::ERROR:
    case State::INIT:
        break;
    }
}

void AP_LTE::bridge_data()
{
    if (!_connected) return;

    uint32_t now = AP_HAL::millis();

    while (_uart_modem->available() && _rx_idx < sizeof(_rx_buf) - 1) {
        _rx_buf[_rx_idx++] = (char)_uart_modem->read();
    }

    if (_rx_idx > 0 &&
        (strstr(_rx_buf, "\r\nRDY\r\n") ||
         strstr(_rx_buf, "+CFUN: 1"))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "LTE_modem: modem reboot -> reconnect");
        _connected = false;
        _rx_idx    = 0;
        memset(_rx_buf, 0, sizeof(_rx_buf));
        change_state(State::WAIT_BOOT);
        return;
    }

    switch (_bridge_state)
    {
    case BridgeState::IDLE:
    {
        if (now - _bridge_state_ms < 50) break;

        uint16_t n = build_mavlink_packet(
            _pending_tx_buf, sizeof(_pending_tx_buf), now);
        if (n > 0) {
            _pending_tx_len = n;
            char cmd[32];
            hal.util->snprintf(cmd, sizeof(cmd),
                               "AT+QISEND=0,%u\r\n", (unsigned)n);
            _uart_modem->write((const uint8_t *)cmd, strlen(cmd));
            _rx_idx = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            _bridge_state    = BridgeState::SEND;
            _bridge_state_ms = now;
            break;
        }

        if (now - _last_poll_ms > 100) {
            _last_poll_ms = now;
            _uart_modem->write((const uint8_t *)"AT+QIRD=0,512\r\n",
                               strlen("AT+QIRD=0,512\r\n"));
            _rx_idx = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            _bridge_state    = BridgeState::READ;
            _bridge_state_ms = now;
        }
        break;
    }

    case BridgeState::SEND:
        if (_rx_idx > 0 && strchr(_rx_buf, '>')) {
            _uart_modem->write(_pending_tx_buf, _pending_tx_len);
            _rx_idx = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            _bridge_state    = BridgeState::IDLE;
            _bridge_state_ms = now;
        } else if (_rx_idx > 0 && strstr(_rx_buf, "ERROR")) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: QISEND ERROR buf=\"%.40s\"", _rx_buf);
            _rx_idx          = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            _bridge_state    = BridgeState::IDLE;
            _bridge_state_ms = now;
        } else if (now - _bridge_state_ms > 3000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE_modem: QISEND timeout buf=\"%.40s\"", _rx_buf);
            _rx_idx          = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            _bridge_state    = BridgeState::IDLE;
            _bridge_state_ms = now;
        }
        break;

    case BridgeState::READ:
        if (_rx_idx > 0 &&
            (strstr(_rx_buf, "\r\nOK") || now - _bridge_state_ms > 1000)) {

            const char *qird = strstr(_rx_buf, "+QIRD: ");
            if (qird) {
                int data_len = atoi(qird + 7);
                if (data_len > 0) {
                    const char *data_start = strchr(qird, '\n');
                    if (data_start) {
                        data_start++;
                        if (_uart_telem) {
                            _uart_telem->write(
                                (const uint8_t *)data_start, data_len);
                        }
                        if (_debug_level >= 1) {
                            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                                          "LTE_modem: QIRD %d B downlink",
                                          data_len);
                        }
                    }
                }
            }
            _rx_idx = 0;
            memset(_rx_buf, 0, sizeof(_rx_buf));
            _bridge_state    = BridgeState::IDLE;
            _bridge_state_ms = now;
        } else if (now - _bridge_state_ms > 1000) {
            _bridge_state    = BridgeState::IDLE;
            _bridge_state_ms = now;
        }
        break;
    }
}

uint16_t AP_LTE::build_mavlink_packet(uint8_t *buf, uint16_t buflen, uint32_t now)
{
    mavlink_message_t msg;
    uint16_t len = 0;

    // HEARTBEAT 1 Hz
    if (now - _mav_last_hb_ms >= 1000) {
        _mav_last_hb_ms = now;
        uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if (hal.util->get_soft_armed()) {
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }
        mavlink_msg_heartbeat_pack(1, 1, &msg,
            MAV_TYPE_FIXED_WING,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode, 0, MAV_STATE_ACTIVE);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        _mav_seq++;
        return len;
    }

    // ATTITUDE 4 Hz
    if (now - _mav_last_att_ms >= 250) {
        _mav_last_att_ms = now;
        const AP_AHRS &ahrs = AP::ahrs();
        Vector3f gyro = ahrs.get_gyro();
        mavlink_msg_attitude_pack(1, 1, &msg,
            AP_HAL::micros(),
            ahrs.get_roll(), ahrs.get_pitch(), ahrs.get_yaw(),
            gyro.x, gyro.y, gyro.z);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        _mav_seq++;
        return len;
    }

    // GLOBAL_POSITION_INT 2 Hz
    if (now - _mav_last_pos_ms >= 500) {
        _mav_last_pos_ms = now;
        const AP_GPS  &gps  = AP::gps();
        const AP_AHRS &ahrs = AP::ahrs();
        Location loc;
        if (!ahrs.get_location(loc)) {
            loc = gps.location();
        }
        Vector3f vel;
        if (!ahrs.get_velocity_NED(vel)) {
            vel.zero();
        }
        float rel_alt_m = 0.0f;
        ahrs.get_relative_position_D_home(rel_alt_m);
        rel_alt_m = -rel_alt_m;
        mavlink_msg_global_position_int_pack(1, 1, &msg,
            AP_HAL::millis(),
            loc.lat, loc.lng,
            loc.alt * 10,
            (int32_t)(rel_alt_m * 1000.0f),
            vel.x * 100, vel.y * 100, vel.z * 100,
            ahrs.yaw_sensor);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        _mav_seq++;
        return len;
    }

    // SYS_STATUS 1 Hz
    if (now - _mav_last_sys_ms >= 1000) {
        _mav_last_sys_ms = now;
        const AP_BattMonitor &battery = AP::battery();
        float voltage_v = battery.voltage();
        float current_a = 0.0f;
        if (!battery.current_amps(current_a)) {
            current_a = -1.0f;
        }
        uint8_t remaining = 0;
        if (!battery.capacity_remaining_pct(remaining)) {
            remaining = 0;
        }
        mavlink_msg_sys_status_pack(1, 1, &msg,
            0, 0, 0, 0,
            (uint16_t)(voltage_v * 1000.0f),
            (int16_t)(current_a * 100.0f),
            (int8_t)remaining,
            0, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        _mav_seq++;
        return len;
    }

    return 0;
}

void AP_LTE::send_at(const char *cmd)
{
    _rx_idx = 0;
    memset(_rx_buf, 0, sizeof(_rx_buf));
    _uart_modem->write((const uint8_t *)cmd, strlen(cmd));
    if (_debug_level >= 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                      "LTE_modem: [%s] >> %s", state_name(_state), cmd);
    }
}

bool AP_LTE::check_response(const char *expected)
{
    if (_rx_idx == 0) return false;
    if (strstr(_rx_buf, expected)) {
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
    if (new_state == State::CONNECTED) {
        _bridge_state    = BridgeState::IDLE;
        _bridge_state_ms = 0;
        _mav_last_hb_ms  = 0;
        _mav_last_att_ms = 0;
        _mav_last_pos_ms = 0;
        _mav_last_sys_ms = 0;
    }
}