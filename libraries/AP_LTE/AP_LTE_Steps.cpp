/*
  Main state machine steps for AP_LTE, ported one for one from the
  step_* functions in LTE_modem.lua.
 */

#include "AP_LTE.h"

#if AP_LTE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

extern const AP_HAL::HAL &hal;

// Pull "<mode>,<stat>" out of a +CEREG:/+CREG: reply. two_field_only rejects
// the single-number URC form, which is what step_CREG wants: it polls and
// reads the answer to its own poll, not the asynchronous notification.
static bool parse_reg(const char *p, int &stat, bool two_field_only)
{
    p = strchr(p, ':');
    if (p == nullptr) {
        return false;
    }
    p++;
    while (*p == ' ') {
        p++;
    }
    int32_t a = 0, b = 0;
    if (!AP_LTE::parse_int(p, a)) {
        return false;
    }
    if (AP_LTE::parse_int_after(p, ',', b)) {
        stat = b;
        return true;
    }
    if (!two_field_only) {
        stat = a;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------

void AP_LTE::step_ATI()
{
    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    const uint32_t now = AP_HAL::millis();
    const uint32_t ati_s = (now - _step_timer_ms) / 1000;
    // Silent on a normal fast boot, but once ATI drags past 2s (modem
    // mid-reboot after a reset) chirp every 5s, so the recovery window is not
    // a black hole in the messages tab.
    if (ati_s > 2 && (_ati_dbg_ms == 0 || now - _ati_dbg_ms > 5000)) {
        _ati_dbg_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: waiting for modem (ATI, %lus)",
                      (unsigned long)ati_s);
    }

    if (_chunk_len > 0 && _modem == &AP_LTE_default_modem) {
        check_modem_banner();
    }
    if (_modem != &AP_LTE_default_modem) {
        _step = Step::BAUD;
        return;
    }

    // The applet alternated a CMUX-framed ATI into this rotation; without that
    // transport the third slot collapses back to a plain ATI.
    if (_ati_sequence % 3 == 2) {
        at_send("+++");
    } else {
        at_send("\rATI\r");
    }

    if (_ati_sequence % 10 == 5) {
        _uart->begin(pget(_baud));
    } else if (_ati_sequence % 10 == 9) {
        _uart->begin(pget(_ibaud));
    }
    _ati_sequence++;
}

void AP_LTE::step_BAUD()
{
    if (_modem->setbaud != nullptr && pget(_baud) != pget(_ibaud)) {
        _change_baud = uint32_t(pget(_baud));
        at_sendf(_modem->setbaud, unsigned(_change_baud));
    }
    _step = Step::CPIN;
}

void AP_LTE::step_CPIN()
{
    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    if (rx_find("READY")) {
        _step = Step::CONFIG;
        return;
    }

    if (rx_find("+QCCID:") || rx_find("+ICCID:")) {
        _sim_probe_count++;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: SIM hardware OK but unready (%u/3)",
                      unsigned(_sim_probe_count));
        if (_sim_probe_count >= 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "LTE FATAL: SIM unresponsive after 3 probes. Halting.");
            _halt_reason = "SIM unresponsive after 3 probes";
            _step = Step::HALT;
            return;
        }
    } else if (rx_find("ERROR: 10") || rx_find("NOT INSERTED") ||
               rx_find("SIM not inserted")) {
        // Debounced like the QCCID path above: a UART bounce can make a modem
        // report the SIM missing for one reply before its SIM interface
        // settles, even with a card seated.
        _sim_missing_count++;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: SIM reported missing, probe %u/3",
                      unsigned(_sim_missing_count));
        if (_sim_missing_count >= 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "LTE FATAL: SIM CARD MISSING! Halting sequence.");
            _halt_reason = "SIM card missing";
            _step = Step::HALT;
            return;
        }
    }

    const uint32_t elapsed_ms = AP_HAL::millis() - _step_timer_ms;

    // first second: ordinary polling for +CPIN: READY
    if (elapsed_ms < 1000) {
        at_send("AT+CPIN?\r\n");
        return;
    }

    // then one probe per second, five of them, before a hard reset
    const uint32_t probe_tick = elapsed_ms / 1000;
    if (probe_tick > _cpin_probe_n) {
        _cpin_probe_n = probe_tick;
        const uint32_t probe_num = probe_tick - 1;
        if (probe_num <= 5) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: SIM not ready, probe %lu/5",
                          (unsigned long)probe_num);
            at_send("AT+CPIN?\r\n");
            at_send(_modem->sim_probe);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "LTE: CPIN failed 5 probes, hard reset");
            reset_to_ATI();
        }
    }
}

void AP_LTE::step_CONFIG()
{
    // Echo off. Harmless on short commands, but the CA cert upload would be
    // mirrored straight back otherwise.
    at_send("ATE0\r\n");
    set_BAND();
    set_MCCMNC();
    at_send(_modem->config_extra);
    if (_modem->fast_connect) {
        at_send("AT+CEREG=1\r\n");
    }
    _step = Step::CREG;
}

void AP_LTE::step_CREG()
{
    uart_read();
    rx_append(_chunk, _chunk_len);

    if (handle_error()) {
        rx_reset();
        return;
    }

    if (_rx_len > 0) {
        const char *p = rx_at("+CEREG:");
        if (p == nullptr) {
            p = rx_at("+CREG:");
        }
        int stat = -1;
        if (p != nullptr && parse_reg(p, stat, true)) {
            if (stat == 1 || stat == 5) {
                rx_reset();
                _cops_zero_sent = false;
                if (_creg_search_ms != 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: CREG OK (search %.1fs)",
                                  (double)(AP_HAL::millis() - _creg_search_ms) * 0.001);
                    _creg_search_ms = 0;
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: CREG OK");
                }
                // Registered. SIMINFO prints the identity block and then makes
                // the HTTPAUTH-or-not decision, so that routing lives in one
                // place rather than both.
                _step = Step::SIMINFO;
                _siminfo_sent = false;
                return;
            }
            if (stat == 0 || stat == 3) {
                rx_reset();
                const uint32_t elapsed_ms = AP_HAL::millis() - _step_timer_ms;
                if (_modem->fast_connect) {
                    if (elapsed_ms >= 2000) {
                        if (!_cops_zero_sent) {
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                          "LTE CREG: not registered, auto-selecting (COPS=0)");
                            at_send("AT+COPS=0\r\n");
                            _cops_zero_sent = true;
                        } else if (elapsed_ms > 12000) {
                            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                          "LTE: CREG search timed out, hard reset");
                            reset_to_ATI();
                            return;
                        }
                    }
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE CREG: not registered");
                    at_send("AT+CFUN=1\r\n");
                    at_send("AT+COPS?\r\n");
                }
                return;
            }
            if (stat == 2) {
                // Printed once per search, not once per 150ms poll: a Cat 1
                // scan can take 10s+, which was ~65 identical lines per boot.
                if (_creg_search_ms == 0) {
                    _creg_search_ms = AP_HAL::millis();
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: CREG searching...");
                }
            }
        }

        // wipe the slate when the modem finishes a sentence
        if (rx_find("OK\r\n") || rx_find("ERROR\r\n")) {
            rx_reset();
        }
    }

    at_send(_modem->fast_connect ? "AT+CEREG?\r\n" : "AT+CREG?\r\n");
}

/*
  One operator, many spellings: "airtel airtel" from EF-SPN against "IND
  airtel" from +COPS, "Jio 4G" against "IND-JIO", Vi as Vodafone or IDEA.
  Reduce to a brand token so the name stops moving with whichever query
  answered first.
 */
static void brand_token(const char *in, char *out, size_t out_len)
{
    out[0] = 0;
    if (in == nullptr || in[0] == 0) {
        return;
    }
    char low[32];
    size_t n = strnlen(in, sizeof(low) - 1);
    for (size_t i = 0; i < n; i++) {
        low[i] = char(tolower(in[i]));
    }
    low[n] = 0;

    struct { const char *needle; const char *brand; } table[] = {
        { "airtel", "airtel" },
        { "jio", "jio" },
        { "vodafone", "vi" },
        { "idea", "vi" },
        { "bsnl", "bsnl" },
        { "mtnl", "mtnl" },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(table); i++) {
        if (strstr(low, table[i].needle) != nullptr) {
            strncpy(out, table[i].brand, out_len - 1);
            out[out_len - 1] = 0;
            return;
        }
    }
    // "vi" needs word boundaries; as a substring it hides inside Movistar
    for (size_t i = 0; i + 1 < n; i++) {
        if (low[i] == 'v' && low[i + 1] == 'i' &&
            (i == 0 || !isalpha(low[i - 1])) &&
            (i + 2 >= n || !isalpha(low[i + 2]))) {
            strncpy(out, "vi", out_len - 1);
            out[out_len - 1] = 0;
            return;
        }
    }

    // Unlisted operator: keep its own name, which is the cue to add it above.
    // Strip the "IND " prefix and the doubled word EF-SPN often carries.
    const char *s = low;
    if (strncmp(s, "ind", 3) == 0 && (s[3] == ' ' || s[3] == '-' || s[3] == '_')) {
        s += 4;
        while (*s == ' ' || *s == '-' || *s == '_') {
            s++;
        }
    }
    size_t take = strlen(s);
    const char *sp = strchr(s, ' ');
    if (sp != nullptr && sp[1] != 0 && strchr(sp + 1, ' ') == nullptr) {
        const size_t first = size_t(sp - s);
        if (strlen(sp + 1) == first && strncmp(s, sp + 1, first) == 0) {
            take = first;
        }
    }
    take = MIN(take, out_len - 1);
    memcpy(out, s, take);
    out[take] = 0;
}

// Extract the contents of the nth quoted field after a token.
static bool quoted_field(const char *p, uint8_t index, char *out, size_t out_len)
{
    for (uint8_t i = 0; i <= index; i++) {
        p = strchr(p, '"');
        if (p == nullptr) {
            return false;
        }
        p++;
        const char *e = strchr(p, '"');
        if (e == nullptr) {
            return false;
        }
        if (i == index) {
            const size_t n = MIN(size_t(e - p), out_len - 1);
            memcpy(out, p, n);
            out[n] = 0;
            return true;
        }
        p = e + 1;
    }
    return false;
}

/*
  Registered but not connected: name the modem and the network, then hand to
  HTTPAUTH. Sits here because AT+COPS? only answers once registered.
  AT+COPS=3,0 pins the read to long alphanumeric, since set_MCCMNC may have
  left it numeric and given "40410" instead of "airtel".
 */
void AP_LTE::step_SIMINFO()
{
    uart_read();
    rx_append(_chunk, _chunk_len);

    if (!_siminfo_sent) {
        at_send("AT+COPS=3,0\r\n");
        at_send("AT+COPS?\r\n");
        at_send(_modem->spn);
        _siminfo_sent = true;
        return;
    }

    // +COPS: <mode>[,<format>,"<oper>"[,<Act>]] -- the registered network
    char oper[20] {};
    const char *p = rx_at("+COPS:");
    const bool have_oper = (p != nullptr) && quoted_field(p, 0, oper, sizeof(oper)) &&
                           oper[0] != 0;

    // The card's own name from EF-SPN. Only the SPN is taken: QSPN's FNN/SNN
    // name the registered network, which +COPS already gives. A blank EF-SPN
    // is normal, not a failure.
    //   +QSPN: "<FNN>","<SNN>","<SPN>",<alphabet>,"<RPLMN>"   (Quectel)
    //   +CSPN: "<SPN>",<display_mode>                         (SimCom)
    char spn[20] {};
    bool have_spn = false;
    const char *q = rx_at("+QSPN:");
    if (q != nullptr) {
        have_spn = quoted_field(q, 2, spn, sizeof(spn)) && spn[0] != 0;
    } else {
        q = rx_at("+CSPN:");
        if (q != nullptr) {
            have_spn = quoted_field(q, 0, spn, sizeof(spn)) && spn[0] != 0;
        }
    }

    // Keyed on the reply prefix, not on a non-empty SPN, or a blank-EF-SPN
    // card burns the whole budget every time. ERROR counts as replied.
    const bool spn_done = (_modem->spn == nullptr) || rx_find("+QSPN:") ||
                          rx_find("+CSPN:") || rx_find("ERROR");

    // Revisits get a shorter budget: restoring the link outranks naming
    // the operator.
    const uint32_t waited_ms = AP_HAL::millis() - _step_timer_ms;
    if (!(have_oper && spn_done) && waited_ms < (_siminfo_printed ? 800U : 1500U)) {
        return;
    }

    // Each field updated only from its own answer: a partial reply must not
    // blank the half that did not arrive, which is the common case at 0.8s.
    if (have_spn) {
        strncpy(_sim_spn, spn, sizeof(_sim_spn) - 1);
        _sim_spn[sizeof(_sim_spn) - 1] = 0;
    }
    if (have_oper) {
        strncpy(_sim_oper, oper, sizeof(_sim_oper) - 1);
        _sim_oper[sizeof(_sim_oper) - 1] = 0;
    }

    // Prefers the card (EF-SPN, fixed) over the registered network.
    brand_token(_sim_spn, _sim_name, sizeof(_sim_name));
    if (_sim_name[0] == 0) {
        brand_token(_sim_oper, _sim_name, sizeof(_sim_name));
    }
    if (_sim_name[0] == 0) {
        strncpy(_sim_name, "unknown", sizeof(_sim_name) - 1);
    }

    _ltei_ms = 0;   // fresh registration: force an LTEI row now

    // Model and firmware cannot change within a boot, so this is the only
    // place either is announced.
    if (!_siminfo_printed) {
        const char *fw = _fw_short[0] != 0 ? _fw_short :
                         (_modem_revision[0] != 0 ? _modem_revision : "unknown");
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: module: %.16s", _modem->name);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: firmware: %.16s", fw);
        _siminfo_printed = true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: sim: %.16s", _sim_name);

    rx_reset();
    if (pget(_httpauth) == 1 && _modem->http != nullptr && !_auth_done) {
        _step = Step::HTTPAUTH;
    } else {
        _step = next_after_registration();
    }
}

void AP_LTE::step_SIGNAL_GATE()
{
    // where to go once the gate is satisfied
    const Step onward = (_modem->cipmode != nullptr)   ? Step::CIPMODE :
                        (_modem->preflight != nullptr) ? Step::QENG :
                                                         Step::SOCKET_STATE;

    if (_post_reset) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: Skipping signal gate post-reset");
        _step = onward;
        return;
    }

    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    bool have_rsrp = false;
    int rsrp = 0;

    const char *p = rx_at("+QCSQ:");
    if (p != nullptr) {
        // +QCSQ: "<sysmode>",<rssi>,<rsrp>,...
        p += strlen("+QCSQ:");
        int32_t rssi = 0, v = 0;
        if (skip_quoted(p) && parse_int_after(p, ',', rssi) &&
            parse_int_after(p, ',', v)) {
            rsrp = int(v);
            have_rsrp = true;
        }
    } else {
        p = rx_at("+CPSI:");
        if (p != nullptr) {
            // the 12th comma separated field
            const char *f = p;
            uint8_t commas = 0;
            while (*f != 0 && commas < 11) {
                if (*f == ',') {
                    commas++;
                }
                f++;
            }
            if (commas == 11) {
                rsrp = int(strtol(f, nullptr, 10));
                have_rsrp = true;
                if (rsrp < -200) {
                    rsrp /= 10;
                }
            }
        }
    }

    if (have_rsrp) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE RSRP=%d dBm", rsrp);
        if (rsrp >= -110) {
            _qcsq_tries = 0;
            _step = onward;
        } else {
            _qcsq_tries++;
            if (_qcsq_tries <= 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE weak RSRP=%d re-scan %u/3",
                              rsrp, unsigned(_qcsq_tries));
                _step = Step::CREG;
            } else {
                _qcsq_tries = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE weak signal - connecting anyway");
                _step = onward;
            }
        }
        return;
    }

    if (_modem->csq_gate != nullptr) {
        at_send(_modem->csq_gate);
    } else {
        _step = onward;
    }
}

void AP_LTE::step_QENG()
{
    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    if (rx_find("+QENG")) {
        handle_AT_reply();
        _step = (_modem->socket_state != nullptr) ? Step::SOCKET_STATE :
                (_modem->cipclose != nullptr)     ? Step::CIPCLOSE :
                                                    Step::CIPOPEN;
        return;
    }
    at_send(_modem->preflight);
}

/*
  Recovery router: once the socket is declared dead, ask whether the radio is
  still registered so each failure goes straight to the right recovery,
  instead of always trying the slow CIPOPEN retry ladder first.
    registered (1/5)  -> socket only  -> SOCKET_STATE, reopen in 1-2s
    not reg (0/2/3/4) -> radio gone   -> close socket, re-register via CREG
    no reply          -> AT wedged    -> hard reset, via the run_step watchdog
 */
void AP_LTE::step_CEREG_CHECK()
{
    uart_read();
    rx_append(_chunk, _chunk_len);

    const char *p = rx_at("+CEREG:");
    int stat = -1;
    if (p != nullptr && parse_reg(p, stat, false)) {
        rx_reset();
        if (stat == 1 || stat == 5) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: registered - socket dead, reopening");
            _cipopen_sent = false;
            _step = (_modem->socket_state != nullptr) ? Step::SOCKET_STATE : Step::CIPOPEN;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "LTE: not registered (CEREG=%d) - re-registering", stat);
            if (_disconnect_ms == 0) {
                _disconnect_ms = AP_HAL::millis();
            }
            at_send(_modem->cipclose);
            _cipopen_sent = false;
            _hard_reset_strikes = 0;
            _step = Step::CREG;
        }
        return;
    }
    at_send(_modem->fast_connect ? "AT+CEREG?\r\n" : "AT+CREG?\r\n");
}

void AP_LTE::step_SOCKET_STATE()
{
    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    if (rx_find("+QISTATE:") || rx_find("+CIPOPEN:") ||
        rx_find("\r\nOK\r\n") || rx_find("\nERROR\r\n")) {
        if (rx_find("+QISTATE: 0") || rx_find("+CIPOPEN: 0,\"TCP\"") ||
            rx_find("+CIPOPEN: 0,\"UDP\"")) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: socket open, closing first");
            _step = Step::CIPCLOSE;
        } else {
            _step = Step::CIPOPEN;
        }
        return;
    }
    if (_modem->socket_state != nullptr) {
        at_send(_modem->socket_state);
    } else {
        _step = Step::CIPOPEN;
    }
}

void AP_LTE::step_CIPCLOSE()
{
    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    if (rx_find("\r\nOK\r\n") || rx_find("\nERROR\r\n") ||
        rx_find("+QICLOSE") || rx_find("+CIPCLOSE")) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: socket closed, opening");
        _step = Step::CIPOPEN;
        return;
    }
    at_send(_modem->cipclose);
}

void AP_LTE::step_CIPMODE()
{
    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    if (rx_find("AT+CACID=0,0")) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: network context set");
        _step = Step::NETOPEN;
        return;
    }
    if (handle_error()) {
        return;
    }
    // A bare OK counts as "transparent mode set" only once we have actually
    // sent the command. Otherwise a reply still in flight from an earlier
    // fire-and-forget command (AT+HTTPTERM on the SimCom auth path) arrives on
    // this first read and is mistaken for it, skipping the real command and
    // leaving the socket non-transparent with no error raised.
    if (_cipmode_sent && (rx_find("\r\r\nOK\r") || rx_find("\r\nOK\r\n"))) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: transparent mode set");
        _step = Step::NETOPEN;
        return;
    }
    at_send(_modem->cipmode);
    _cipmode_sent = true;
}

void AP_LTE::step_NETOPEN()
{
    const Step onward = (_modem->preflight != nullptr) ? Step::QENG :
                        _modem->fast_connect           ? Step::SOCKET_STATE :
                                                         Step::CIPOPEN;
    if (_modem->netopen == nullptr) {
        _step = onward;
        return;
    }

    uart_read();
    rx_reset();
    rx_append(_chunk, _chunk_len);

    if (rx_find("AT+CNACT=0,1") && rx_find("ERROR") && _modem->netclose != nullptr) {
        at_send(_modem->netclose);
        return;
    }
    if (handle_error()) {
        return;
    }
    if (rx_find("NETOPEN\r") || rx_find("ACTIVE\r") || rx_find("OK\r")) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: network opened");
        _step = onward;
        return;
    }
    at_send(_modem->netopen);
}

void AP_LTE::step_CIPOPEN()
{
    uart_read();
    rx_append(_chunk, _chunk_len);

    // The applet also tried to set _cipopen_preclosed from the previous step
    // here, but run_step latches last_step before dispatching, so that test
    // could never be true and the flag was only ever set by the pre-close
    // branch below. Not carried over.

    if (_rx_len == 0 && _modem->cipclose != nullptr && !_cipopen_sent &&
        !_cipopen_preclosed) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: pre-close stale socket");
        _cipopen_preclosed = true;
        _step = Step::CIPCLOSE;
        return;
    }

    if (_rx_len > 0) {
        const bool caopen_ok = rx_find("+CAOPEN: 0,0") && rx_find("OK\r\n");
        if (caopen_ok && _modem->caswitch != nullptr) {
            at_send(_modem->caswitch);
            rx_reset();
            return;
        }
        if (rx_find("CONNECT") || rx_find("+QIOPEN: 0,0") ||
            rx_find("+CIPOPEN: 0,0") || caopen_ok) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: connected");
            _cipopen_sent = false;
            _cipopen_retry = 0;
            _hard_reset_strikes = 0;
            _cipopen_preclosed = false;
            reset_buffers();
            _step = Step::CONNECTED;
            return;
        }
    }

    const bool is_error = rx_find("\nERROR\r\n");
    if (is_error) {
        rx_reset();
    }

    if (_cipopen_sent) {
        const uint32_t wait = (_cipopen_sent_ms > 0) ?
                              (AP_HAL::millis() - _cipopen_sent_ms) : 0;
        if (wait > 3000 || is_error) {
            _cipopen_retry++;
            _cipopen_sent = false;
            rx_reset();
            if (_cipopen_retry <= 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE CIPOPEN timeout/error retry %u/3",
                              unsigned(_cipopen_retry));
                if (_modem->cipclose != nullptr) {
                    _step = Step::CIPCLOSE;
                }
            } else {
                _hard_reset_strikes++;
                if (_hard_reset_strikes >= 3) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                                  "LTE CIPOPEN completely dead - HARD RESET");
                    _hard_reset_strikes = 0;
                    reset_to_ATI();
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                                  "LTE CIPOPEN failed 3x - soft reset to CREG");
                    _cipopen_retry = 0;
                    _cipopen_preclosed = false;
                    _step = Step::CREG;
                }
            }
        }
        return;
    }

    if (is_error) {
        return;
    }

    uint8_t ip[4];
    uint16_t port = 0;
    effective_server(ip, port);
    if (port == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Must set LTE_SERVER_PORT");
        return;
    }

    const char *cipopen = option_enabled(OPT_TCP) ? _modem->cipopen_tcp
                                                  : _modem->cipopen_udp;
    if (cipopen == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE: modem has no socket-open command");
        _halt_reason = "modem has no socket-open command";
        _step = Step::HALT;
        return;
    }
    at_sendf(cipopen, int(ip[0]), int(ip[1]), int(ip[2]), int(ip[3]), int(port));
    _cipopen_sent = true;
    _cipopen_sent_ms = AP_HAL::millis();
}

void AP_LTE::step_CONNECTED()
{
    // Anything the virtual port could not take last pass goes first, and the
    // modem UART is left alone until it has.
    flush_fc();

    _chunk_len = 0;
    if (_fc_len == 0) {
        const ssize_t r = _uart->read(_chunk, sizeof(_chunk));
        if (r > 0) {
            _chunk_len = uint16_t(r);
            _bytes_in += _chunk_len;
            if (option_enabled(OPT_LOGALL)) {
                log_raw(_chunk, _chunk_len, "<<<");
            }
        }
    }

    if (chunk_find("\r\nCLOSED\r\n")) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: connection closed, reconnecting");
        _cipopen_sent = false;
        _step = Step::CIPOPEN;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (_chunk_len > 0) {
        _last_data_ms = now_ms;

        // downlink to the vehicle
        const size_t written = _port.device_write(_chunk, _chunk_len);
        if (written < _chunk_len) {
            const uint16_t left = uint16_t(_chunk_len - written);
            _fc_len = MIN(left, uint16_t(sizeof(_fc)));
            memcpy(_fc, &_chunk[written], _fc_len);
        }

        // Scan the same bytes for AT URCs. In transparent mode the socket
        // payload and the modem's own notifications share one stream, so this
        // buffer holds telemetry too and every search over it must be binary
        // safe.
        rx_append(_chunk, _chunk_len);
        if (rx_find("OK\r\n") || rx_find("ERROR\r\n")) {
            handle_AT_reply();
            if (rx_find("OK\r\n")) {
                rx_consume_through("OK\r\n");
            } else {
                rx_consume_through("ERROR\r\n");
            }
        }

        if (chunk_find("+CEREG: 0") || chunk_find("+CEREG: 2") ||
            chunk_find("+CREG: 0,0") || chunk_find("+CREG: 0,2")) {
            if (_cereg_drop_ms == 0) {
                _cereg_drop_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: CEREG lost - %ds grace",
                              int(pget(_grace)));
            }
        } else if (chunk_find("+CEREG: 1") || chunk_find("+CEREG: 5") ||
                   chunk_find("+CREG: 0,1") || chunk_find("+CREG: 0,5")) {
            _cereg_drop_ms = 0;
        }

    } else if (pget(_timeout) > 0 &&
               now_ms - _last_data_ms > uint32_t(pget(_timeout)) * 1000) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LTE_modem: data timeout");
        if (_disconnect_ms == 0) {
            _disconnect_ms = now_ms - uint32_t(pget(_timeout)) * 1000;
        }
        reset_to_ATI();
        return;
    }

    if (_cereg_drop_ms != 0 &&
        now_ms - _cereg_drop_ms > uint32_t(pget(_grace)) * 1000) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE: network lost - fast reconnect");
        if (_disconnect_ms == 0) {
            _disconnect_ms = _cereg_drop_ms;
        }
        _cereg_drop_ms = 0;
        _cipopen_sent = false;
        _hard_reset_strikes = 0;
        at_send(_modem->cipclose);
        _step = Step::CREG;
        return;
    }

    // uplink, vehicle -> modem
    uint32_t quota = 0;
    if (pget(_tx_rate) > 0) {
        quota = uint32_t((now_ms - _last_send_data_ms) * 0.001f * pget(_tx_rate));
    }
    uint32_t sent = 0;
    while (_port.device_available() > 0) {
        uint16_t n = 100;
        if (quota > 0) {
            if (sent >= quota) {
                break;
            }
            n = MIN(n, uint16_t(quota - sent));
        }
        if (_txq == nullptr || _txq->space() < n) {
            break;
        }
        uint8_t tmp[100];
        const ssize_t got = _port.device_read(tmp, n);
        if (got <= 0) {
            break;
        }
        send_raw(tmp, uint16_t(got));
        sent += uint32_t(got);
        _last_send_data_ms = now_ms;
    }

    if (option_enabled(OPT_NOSIGQUERY)) {
        return;
    }

    if (now_ms - _last_CSQ_ms > 1500) {
        _last_CSQ_ms = now_ms;
        at_send(_modem->cpsi != nullptr ? _modem->cpsi : "AT+CSQ\r\n");
    }
    if (now_ms - _last_CSQ_reply_ms > 5000) {
        _last_CSQ_reply_ms = now_ms;
        gcs().send_named_float("LTE_RSSI", -1);
    }
    if (pget(_mccmnc) != _last_mccmnc && _modem->mccmnc != nullptr) {
        set_MCCMNC();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: set MCCMNC=%ld", long(_last_mccmnc));
        _step = Step::CREG;
    }
    if (pget(_band) != _last_band &&
        (_modem->setband != nullptr || _modem->setband_mask != nullptr)) {
        set_BAND();
        if (_last_band != 0) {
            _step = Step::CREG;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE_modem: set BAND=%ld", long(_last_band));
    }
}

#endif  // AP_LTE_ENABLED
