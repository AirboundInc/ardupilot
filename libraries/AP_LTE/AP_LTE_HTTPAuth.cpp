/*
  HTTPS auth step for AP_LTE: fetch the relay server address, and with it the
  sysId and MAVLink signing key, from the auth endpoint over TLS with full
  certificate verification.

  There is no static fallback. A login that cannot be completed halts the
  driver rather than connecting somewhere unverified.
 */

#include "AP_LTE.h"

#if AP_LTE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CustomStorage/AP_CustomStorage.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

// Cap on the whole auth attempt. Real logins routinely take 10-12s and there
// is no useful static fallback, so this is biased toward completing.
#define HTTP_TOTAL_TIMEOUT 25000

// Granted only when a cert upload actually happens, on a never-provisioned or
// freshly wiped module. With the cert already present HTTPAUTH finishes in
// 3-4s; a from-scratch upload plus the normal login after it can run past the
// shared budget even though every step along the way completes fine.
#define CERT_UPLOAD_EXTRA_TIMEOUT 25000

#define HTTPAUTH_MAX_RETRIES 3

/*
  Login body from the per-aircraft craft ID and password in custom storage.
  Craft ID is written onto each board directly, in the form "AA-TRT-00659".
  Returns false if nothing has been provisioned; there is no fallback.
 */
bool AP_LTE::build_auth_body()
{
    _auth_body[0] = 0;
#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE == 1
    AP_CustomStorage *cs = AP_CustomStorage::get_singleton();
    if (cs == nullptr) {
        return false;
    }
    char craft_id[CUSTOM_PARAM_CRAFT_ID_LEN + 1] {};
    char password[CUSTOM_PARAM_PASS_LEN + 1] {};
    if (!cs->get_craft_id(craft_id, sizeof(craft_id)) ||
        !cs->get_password(password, sizeof(password))) {
        return false;
    }
    // Deliberate deviation from the applet: its binding returned an empty
    // string rather than nil for an initialised but blank board, so a wiped
    // vehicle posted "aircraftId=&password=" and only halted after the server
    // rejected it three times. Same end state, ~75s sooner and with a reason
    // that names the actual problem.
    if (craft_id[0] == 0 || password[0] == 0) {
        return false;
    }
    hal.util->snprintf(_auth_body, sizeof(_auth_body), "aircraftId=%s&password=%s",
                       craft_id, password);
    return true;
#else
    return false;
#endif
}

/*
  reason names what failed. no_retry marks failures no retry can fix, such as
  nothing being provisioned to send: those skip straight to HALT rather than
  burning the retry budget.
 */
void AP_LTE::http_fail(const char *reason, bool no_retry)
{
    if (no_retry || _http_retry_count >= HTTPAUTH_MAX_RETRIES) {
        // Close the session best effort without waiting for the reply: HALT is
        // terminal and nothing downstream parses AT replies in a way a stray
        // OK can derail. No-op on Quectel, which has no term command.
        if (_modem->http != nullptr) {
            at_send(_modem->http->term);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "LTE HTTPAUTH: %s - no static fallback, halting", reason);
        _auth_port = 0;
        _auth_done = true;
        _http_sub = HttpSub::NONE;
        _halt_reason = "HTTPAUTH failed";
        _step = Step::HALT;
        return;
    }

    _http_retry_count++;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: %s - retry %u/%u",
                  reason, unsigned(_http_retry_count), unsigned(HTTPAUTH_MAX_RETRIES));

    if (_modem->http != nullptr && _modem->http->term != nullptr) {
        // SimCom needs an explicit AT+HTTPTERM or the retry's HTTPINIT piles
        // on top of the failed session. Awaited in its own sub-state rather
        // than fired and forgotten: the reply is a bare OK, which is also what
        // SC_CERT_LIST reads as "cert list empty", so racing it leaves every
        // later reply one command out of step. The deadline is refreshed --
        // TERM_WAIT starts the new attempt, it is not the tail of the old one.
        _http_sub = HttpSub::TERM_WAIT;
        rx_reset();
        _http_deadline = AP_HAL::millis() + HTTP_TOTAL_TIMEOUT;
        at_send(_modem->http->term);
    } else {
        // Quectel: no session to close. NONE is step_HTTPAUTH's own
        // "start a fresh attempt" signal.
        _http_sub = HttpSub::NONE;
    }
}

void AP_LTE::step_HTTPAUTH()
{
    const AP_LTE_HTTP *http = _modem->http;
    if (http == nullptr) {
        _step = next_after_registration();
        return;
    }

    if (_http_sub == HttpSub::NONE) {
        // Computed once per attempt: the length sent in QHTTPPOST and the
        // bytes sent afterward have to match exactly.
        if (!build_auth_body()) {
            // retrying cannot conjure up provisioning
            http_fail("no craft ID/password provisioned in custom storage", true);
            return;
        }
        // The cert presence check always runs first, for both dialects. The
        // cert file survives a modem reset, so this is only a real upload on a
        // never-provisioned or freshly wiped module; every other entry into
        // HTTPAUTH just lists and moves on.
        _http_sub = http->simcom ? HttpSub::SC_CERT_LIST : HttpSub::CERT_LIST;
        _http_cfg_i = 0;
        rx_reset();
        _http_deadline = AP_HAL::millis() + HTTP_TOTAL_TIMEOUT;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: fetching server addr");
        at_send(http->cert_list);
    }

    if (int32_t(AP_HAL::millis() - _http_deadline) > 0) {
        if (_http_sub == HttpSub::TERM_WAIT) {
            // A modem that will not answer a teardown has a wedged HTTP
            // session, so reset it rather than retry. That discards the
            // session and anything still in flight with it, which no amount of
            // retrying can do.
            if (_term_reset_done) {
                http_fail("HTTPTERM timeout", true);
                return;
            }
            _term_reset_done = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: HTTPTERM timeout, modem reset");
            reset_to_ATI();
            return;
        }
        http_fail("timeout");
        return;
    }

    uart_read();
    rx_append(_chunk, _chunk_len);

    const bool ok = rx_find("OK\r\n");
    const bool err = rx_find("ERROR");

    switch (_http_sub) {

    case HttpSub::NONE:
        break;

    case HttpSub::TERM_WAIT:
        // Exactly one command is outstanding, the AT+HTTPTERM http_fail() just
        // sent, so the first OK or ERROR here is unambiguously its reply --
        // ERROR merely means there was no session to stop. Consuming it is the
        // whole point of this state; clearing the sub-state hands back to the
        // fresh-attempt block above.
        if (ok || err) {
            rx_reset();
            _http_sub = HttpSub::NONE;
        }
        break;

    // ---- Quectel ----------------------------------------------------------

    case HttpSub::CERT_LIST:
        // AT+QFLST="*" echoes existing files as "UFS:<name>",<size>, so a
        // substring match on the filename is enough to know it is there.
        if (rx_find(http->cert_filename)) {
            rx_reset();
            _http_sub = HttpSub::CFG;
            _http_cfg_i = 0;
        } else if (ok || err) {
            rx_reset();
            _http_sub = HttpSub::CERT_UPLOAD_CMD;
            _http_deadline += CERT_UPLOAD_EXTRA_TIMEOUT;
            at_sendf(http->cert_upload, unsigned(AP_LTE_ca_cert_len));
        }
        break;

    case HttpSub::CERT_UPLOAD_CMD:
        if (rx_find("CONNECT")) {
            rx_reset();
            send_raw((const uint8_t *)AP_LTE_ca_cert_pem, AP_LTE_ca_cert_len);
            _http_sub = HttpSub::CERT_UPLOAD_DATA;
        } else if (err) {
            http_fail("cert upload error");
        }
        break;

    case HttpSub::CERT_UPLOAD_DATA:
        // The module replies "+QFUPL: <size>,<crc>" then OK. The CRC is not
        // decoded: a bad upload just fails the handshake downstream at
        // seclevel=1 and the fix is the same either way, a re-upload. It is in
        // the raw log if a mismatch ever needs diagnosing by hand.
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::CFG;
            _http_cfg_i = 0;
        } else if (err) {
            http_fail("cert upload data error");
        }
        break;

    case HttpSub::CFG:
        // one config command per OK/ERROR, then on to context activation
        if (_http_cfg_i == 0 || ok || err) {
            const char *cmd = http->cfg[_http_cfg_i];
            _http_cfg_i++;
            rx_reset();
            if (cmd != nullptr) {
                at_send(cmd);
            } else {
                _http_sub = HttpSub::CACERT;
                at_send(http->cacert);
            }
        }
        break;

    case HttpSub::CACERT:
        // Hard fail, not tolerated: this binds the CA cert that seclevel=1
        // verifies against. Continuing on ERROR would let the login proceed
        // over a connection that is not actually verified.
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SECLEVEL;
            at_send(http->seclevel);
        } else if (err) {
            http_fail("cacert bind error");
        }
        break;

    case HttpSub::SECLEVEL:
        // Hard fail: this is the command that actually turns on server cert
        // verification. Same reasoning as CACERT.
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::ACT;
            at_send(http->act);
        } else if (err) {
            http_fail("seclevel set error");
        }
        break;

    case HttpSub::ACT:
        if (ok || err) {    // ERROR here means already active, which is fine
            rx_reset();
            _http_sub = HttpSub::URL_CMD;
            at_sendf(http->url, unsigned(strlen(AP_LTE_auth_url)));
        }
        break;

    case HttpSub::URL_CMD:
        if (rx_find("CONNECT")) {
            rx_reset();
            at_send(AP_LTE_auth_url);   // exactly the URL, no CRLF
            _http_sub = HttpSub::URL_DATA;
        } else if (err) {
            http_fail("QHTTPURL error");
        }
        break;

    case HttpSub::URL_DATA:
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::POST_CMD;
            at_sendf(http->post, unsigned(strlen(_auth_body)));
        } else if (err) {
            http_fail("URL data error");
        }
        break;

    case HttpSub::POST_CMD:
        if (rx_find("CONNECT")) {
            rx_reset();
            at_send(_auth_body);
            _http_sub = HttpSub::POST_DATA;
        } else if (err) {
            http_fail("QHTTPPOST error");
        }
        break;

    case HttpSub::POST_DATA: {
        const char *p = rx_at("+QHTTPPOST:");
        if (p != nullptr) {
            p += strlen("+QHTTPPOST:");
            int32_t perr = -1, code = 0;
            if (parse_int(p, perr) && parse_int_after(p, ',', code) &&
                perr == 0 && code >= 200 && code < 300) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: login OK (%ld)", long(code));
                rx_reset();
                _http_sub = HttpSub::READ;
                at_send(http->read);
            } else {
                char reason[48];
                hal.util->snprintf(reason, sizeof(reason), "POST err=%ld http=%ld", long(perr), long(code));
                http_fail(reason);
            }
        } else if (err) {
            http_fail("POST data error");
        }
        break;
    }

    case HttpSub::READ: {
        if (rx_find("+QHTTPREAD:") || (rx_find("CONNECT") && ok)) {
            const char *body = _rx;
            const char *c1 = rx_at("CONNECT");
            if (c1 != nullptr) {
                body = c1 + 7;      // payload sits between CONNECT and the trailing OK
            }
            uint8_t ip[4];
            uint16_t port = 0;
            if (parse_auth_response(body, ip, port) && port > 0) {
                memcpy(_auth_ip, ip, 4);
                _auth_port = port;
                _auth_done = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: server %u.%u.%u.%u:%u",
                              ip[0], ip[1], ip[2], ip[3], unsigned(port));
                apply_auth_identity(body);
                _http_sub = HttpSub::NONE;
                rx_reset();
                _step = next_after_registration();
            } else {
                http_fail("parse failed");
            }
        } else if (err) {
            http_fail("QHTTPREAD error");
        }
        break;
    }

    // ---- SimCom -----------------------------------------------------------
    //
    // Same overall shape, but SimCom takes the URL and content type as
    // HTTPPARA settings instead of a separate upload, and reports the result
    // asynchronously via a +HTTPACTION URC rather than inline after the POST.

    case HttpSub::SC_CERT_LIST:
        // AT+CCERTLIST reports only the filename: unlike QFLST there is no
        // integrity signal here, just presence.
        if (rx_find(http->cert_filename)) {
            rx_reset();
            _http_sub = HttpSub::SC_INIT;
            at_send(http->init);
        } else if (ok || err) {
            rx_reset();
            _http_sub = HttpSub::SC_CERT_UPLOAD_CMD;
            _http_deadline += CERT_UPLOAD_EXTRA_TIMEOUT;
            at_sendf(http->cert_upload, unsigned(AP_LTE_ca_cert_len));
        }
        break;

    case HttpSub::SC_CERT_UPLOAD_CMD:
        if (rx_find(">")) {     // prompt here is a bare ">", not Quectel's CONNECT
            rx_reset();
            send_raw((const uint8_t *)AP_LTE_ca_cert_pem, AP_LTE_ca_cert_len);
            _http_sub = HttpSub::SC_CERT_UPLOAD_DATA;
        } else if (err) {
            http_fail("cert upload error");
        }
        break;

    case HttpSub::SC_CERT_UPLOAD_DATA:
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SC_INIT;
            at_send(http->init);
        } else if (err) {
            http_fail("cert upload data error");
        }
        break;

    case HttpSub::SC_INIT:
        if (ok || err) {    // ERROR tolerated: nothing was open to init over
            rx_reset();
            _http_sub = HttpSub::SC_CID;
            at_send(http->cid);
        }
        break;

    case HttpSub::SC_CID:
        if (ok || err) {
            rx_reset();
            _http_sub = HttpSub::SC_CACERT;
            at_send(http->cacert);
        }
        break;

    case HttpSub::SC_CACERT:
        // Hard fail: binds the CA cert that authmode=1 verifies against.
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SC_SSL;
            at_send(http->ssl);
        } else if (err) {
            http_fail("cacert bind error");
        }
        break;

    case HttpSub::SC_SSL:
        // Hard fail: authmode=1 is what actually turns verification on. A CA
        // bound without it loads but is never consulted.
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SC_URL;
            at_send(http->url);
        } else if (err) {
            http_fail("authmode set error");
        }
        break;

    case HttpSub::SC_URL:
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SC_CONTENT;
            at_send(http->content);
        } else if (err) {
            http_fail("HTTPPARA URL error");
        }
        break;

    case HttpSub::SC_CONTENT:
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SC_DATA_CMD;
            at_sendf(http->data, unsigned(strlen(_auth_body)));
        } else if (err) {
            http_fail("HTTPPARA CONTENT error");
        }
        break;

    case HttpSub::SC_DATA_CMD:
        if (rx_find("DOWNLOAD")) {
            rx_reset();
            at_send(_auth_body);
            _http_sub = HttpSub::SC_DATA_BODY;
        } else if (err) {
            http_fail("HTTPDATA error");
        }
        break;

    case HttpSub::SC_DATA_BODY:
        if (ok) {
            rx_reset();
            _http_sub = HttpSub::SC_ACTION;
            at_send(http->action);
        } else if (err) {
            http_fail("HTTPDATA body error");
        }
        break;

    case HttpSub::SC_ACTION: {
        const char *p = rx_at("+HTTPACTION:");
        if (p != nullptr) {
            p += strlen("+HTTPACTION:");
            int32_t method = 0, code = 0, len = 0;
            if (parse_int(p, method) && parse_int_after(p, ',', code) &&
                parse_int_after(p, ',', len) && code >= 200 && code < 300) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: login OK (%ld)", long(code));
                rx_reset();
                _http_sub = HttpSub::SC_READ;
                at_sendf(http->read, unsigned(MAX(len, 1)));
            } else {
                char reason[32];
                hal.util->snprintf(reason, sizeof(reason), "POST http=%ld", long(code));
                http_fail(reason);
            }
        } else if (err) {
            http_fail("HTTPACTION error");
        }
        break;
    }

    case HttpSub::SC_READ:
        if (rx_find("+HTTPREAD:") && rx_find("\r\nOK\r\n")) {
            uint8_t ip[4];
            uint16_t port = 0;
            if (parse_auth_response(_rx, ip, port) && port > 0) {
                memcpy(_auth_ip, ip, 4);
                _auth_port = port;
                _auth_done = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: server %u.%u.%u.%u:%u",
                              ip[0], ip[1], ip[2], ip[3], unsigned(port));
                apply_auth_identity(_rx);
                at_send(http->term);    // best effort cleanup, reply not awaited
                _http_sub = HttpSub::NONE;
                rx_reset();
                _step = next_after_registration();
            } else {
                http_fail("parse failed");
            }
        } else if (err) {
            http_fail("HTTPREAD error");
        }
        break;
    }
}

#endif  // AP_LTE_ENABLED
