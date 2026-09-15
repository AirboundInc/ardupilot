/*
  AT reply parsers and the auth-response handling for AP_LTE.
 */

#include "AP_LTE.h"

#if AP_LTE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

extern const AP_HAL::HAL &hal;

/*
  Strict decimal parse: anything that is not wholly digits (with an optional
  sign) yields the fallback, which is what Lua tonumber() did. Several fields
  below are documented as hex by the vendor but were read this way by the
  applet, so a hex cell ID has always logged as the fallback; keeping the
  behaviour keeps the logs comparable across the port.
 */
static int32_t dec_or(const char *s, uint16_t len, int32_t fallback)
{
    if (len == 0) {
        return fallback;
    }
    uint16_t i = 0;
    bool neg = false;
    if (s[0] == '-' || s[0] == '+') {
        neg = (s[0] == '-');
        i = 1;
        if (len == 1) {
            return fallback;
        }
    }
    int64_t v = 0;
    for (; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return fallback;
        }
        v = v * 10 + (s[i] - '0');
        if (v > INT32_MAX) {
            return fallback;
        }
    }
    return int32_t(neg ? -v : v);
}

static int32_t hex_or(const char *s, uint16_t len, int32_t fallback)
{
    if (len == 0) {
        return fallback;
    }
    int64_t v = 0;
    for (uint16_t i = 0; i < len; i++) {
        const char c = s[i];
        int d;
        if (c >= '0' && c <= '9') {
            d = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            d = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            d = c - 'A' + 10;
        } else {
            return fallback;
        }
        v = v * 16 + d;
        if (v > INT32_MAX) {
            return fallback;
        }
    }
    return int32_t(v);
}

// Split a comma separated run into field pointers. Stops at CR, LF or NUL.
// Empty fields are preserved, which matters: the applet's CSV handling had to
// respect empty commas or every later telemetry column shifted.
static uint8_t csv_split(const char *s, const char **out, uint16_t *lens,
                         uint8_t max_fields)
{
    uint8_t n = 0;
    const char *start = s;
    while (n < max_fields) {
        const char *p = start;
        while (*p != 0 && *p != ',' && *p != '\r' && *p != '\n') {
            p++;
        }
        out[n] = start;
        lens[n] = uint16_t(p - start);
        n++;
        if (*p != ',') {
            break;
        }
        start = p + 1;
    }
    return n;
}

// strip surrounding double quotes in place of the pointer/length pair
static void unquote(const char *&s, uint16_t &len)
{
    if (len >= 2 && s[0] == '"' && s[len - 1] == '"') {
        s++;
        len -= 2;
    }
}

static void copy_field(char *dst, size_t dst_len, const char *src, uint16_t src_len)
{
    const size_t n = MIN(size_t(src_len), dst_len - 1);
    memcpy(dst, src, n);
    dst[n] = 0;
}

// ---------------------------------------------------------------------------
// scanning helpers, shared with the step and auth files
// ---------------------------------------------------------------------------

void AP_LTE::skip_ws(const char *&p)
{
    while (*p == ' ' || *p == '\t') {
        p++;
    }
}

bool AP_LTE::parse_int(const char *&p, int32_t &v)
{
    skip_ws(p);
    const char *start = p;
    bool neg = false;
    if (*p == '-' || *p == '+') {
        neg = (*p == '-');
        p++;
    }
    if (!isdigit(*p)) {
        p = start;
        return false;
    }
    int64_t acc = 0;
    while (isdigit(*p)) {
        acc = acc * 10 + (*p - '0');
        if (acc > INT32_MAX) {
            p = start;
            return false;
        }
        p++;
    }
    v = int32_t(neg ? -acc : acc);
    return true;
}

bool AP_LTE::expect_char(const char *&p, char c)
{
    skip_ws(p);
    if (*p != c) {
        return false;
    }
    p++;
    return true;
}

bool AP_LTE::parse_int_after(const char *&p, char sep, int32_t &v)
{
    return expect_char(p, sep) && parse_int(p, v);
}

bool AP_LTE::parse_ipv4(const char *&p, uint8_t ip[4])
{
    const char *start = p;
    int32_t o[4];
    for (uint8_t i = 0; i < 4; i++) {
        if (i > 0 && !expect_char(p, '.')) {
            p = start;
            return false;
        }
        if (!parse_int(p, o[i]) || o[i] < 0 || o[i] > 255) {
            p = start;
            return false;
        }
    }
    for (uint8_t i = 0; i < 4; i++) {
        ip[i] = uint8_t(o[i]);
    }
    return true;
}

bool AP_LTE::skip_quoted(const char *&p)
{
    if (!expect_char(p, '"')) {
        return false;
    }
    const char *close = strchr(p, '"');
    if (close == nullptr) {
        return false;
    }
    p = close + 1;
    return true;
}

// ---------------------------------------------------------------------------
// signal parsers
// ---------------------------------------------------------------------------

bool AP_LTE::check_CSQ()
{
    const char *p = rx_at("+CSQ:");
    if (p == nullptr) {
        return false;
    }
    p += strlen("+CSQ:");
    int32_t rssi = 0, ber = 0;
    if (!parse_int(p, rssi) || !parse_int_after(p, ',', ber)) {
        return false;
    }
    gcs().send_named_float("LTE_RSSI", rssi);
    AP::logger().Write("LTE", "RSSI,BER,Bin,Bout", "iiII",
                       rssi, ber, _bytes_in, _bytes_out);
    return true;
}

bool AP_LTE::check_CPSI()
{
    const char *p = rx_at("+CPSI:");
    if (p == nullptr) {
        return false;
    }

    char r1[65] {};
    char r2[65] {};
    copy_field(r1, sizeof(r1), _rx, MIN(_rx_len, uint16_t(64)));
    if (_rx_len > 64) {
        copy_field(r2, sizeof(r2), &_rx[64], MIN(uint16_t(_rx_len - 64), uint16_t(64)));
    }
    AP::logger().Write("LTER", "R1,R2", "--", "--", "ZZ", r1, r2);

    p += strlen("+CPSI:");
    while (*p == ' ') {
        p++;
    }

    const char *f[16];
    uint16_t l[16];
    const uint8_t n = csv_split(p, f, l, 14);
    if (n < 14) {
        return false;
    }

    // Field order as the applet read it: the two quality fields arrive
    // RSRQ then RSRP on this dialect.
    char mode[17], oper[17], mccmnc[17], band_str[17];
    copy_field(mode, sizeof(mode), f[0], l[0]);
    copy_field(oper, sizeof(oper), f[1], l[1]);
    copy_field(mccmnc, sizeof(mccmnc), f[2], l[2]);
    copy_field(band_str, sizeof(band_str), f[6], l[6]);

    int32_t tac = 0;
    if (l[3] > 2 && f[3][0] == '0' && (f[3][1] == 'x' || f[3][1] == 'X')) {
        tac = hex_or(f[3] + 2, l[3] - 2, 0);
    } else {
        tac = dec_or(f[3], l[3], 0);
    }
    const int32_t scell_id = dec_or(f[4], l[4], 0);
    const int32_t pcid     = dec_or(f[5], l[5], 0);
    const int32_t ul_freq  = dec_or(f[7], l[7], 0);
    const int32_t dl_freq  = dec_or(f[8], l[8], 0);
    const int32_t tdd_cfg  = dec_or(f[9], l[9], 0);
    const int32_t rsrq     = dec_or(f[10], l[10], 0);
    const int32_t rsrp     = dec_or(f[11], l[11], 0);
    const int32_t rssi     = dec_or(f[12], l[12], 0);
    const int32_t sinr     = dec_or(f[13], l[13], 0);

    // "EUTRAN-BAND1" -> 1: the digits that follow the last non-digit run
    int32_t band = -1;
    for (uint16_t i = 0; i < l[6]; i++) {
        if (f[6][i] >= '0' && f[6][i] <= '9') {
            band = dec_or(&f[6][i], uint16_t(l[6] - i), -1);
            break;
        }
    }

    // units/mults spelled out: see the note on the LTEI write in AP_LTE.cpp.
    // Leading string data would otherwise pick the five-string overload and
    // send the operator name through as the format.
    AP::logger().Write("LTES", "Md,Op,MCC,TAC,CID,PID,BND,F,DF,TDD,RP,RQ,RS,SR",
                       "--------------", "--------------",
                       "NNNIIINHhhhhhh",
                       mode, oper, mccmnc,
                       uint32_t(tac), uint32_t(scell_id), uint32_t(pcid),
                       band_str,
                       uint16_t(ul_freq), int16_t(dl_freq), int16_t(tdd_cfg),
                       int16_t(rsrp), int16_t(rsrq), int16_t(rssi), int16_t(sinr));

    if (option_enabled(OPT_SIGNALS)) {
        gcs().send_named_float("LTE_RSRP", rsrp);
        gcs().send_named_float("LTE_RSRQ", rsrq);
        gcs().send_named_float("LTE_BAND", band);
        gcs().send_named_float("LTE_CID", scell_id >> 8);
        const char *mp = mccmnc;
        int32_t mcc = 0, mnc = 0;
        if (parse_int(mp, mcc) && parse_int_after(mp, '-', mnc)) {
            gcs().send_named_float("LTE_MCCMNC", mcc * 100 + mnc);
        }
    }

    const int32_t tower_id = scell_id >> 8;
    if (_track_band < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: connected on Band %ld (%s)",
                      long(band), band_str);
        _track_band = band;
    } else if (band != _track_band) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE WARNING: band switch %ld -> %ld (%s)",
                      long(_track_band), long(band), band_str);
        _track_band = band;
    }
    if (_track_cid < 0) {
        _track_cid = tower_id;
    } else if (tower_id != _track_cid) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE WARNING: cell tower switch CID %ld -> %ld",
                      long(_track_cid), long(tower_id));
        _track_cid = tower_id;
    }
    return true;
}

bool AP_LTE::check_QENG()
{
    const char *p = rx_at("+QENG:");
    if (p == nullptr) {
        return false;
    }
    const char *sc = rx_at("\"servingcell\"");
    if (sc == nullptr) {
        return false;
    }

    char r1[65] {};
    char r2[65] {};
    copy_field(r1, sizeof(r1), _rx, MIN(_rx_len, uint16_t(64)));
    if (_rx_len > 64) {
        copy_field(r2, sizeof(r2), &_rx[64], MIN(uint16_t(_rx_len - 64), uint16_t(64)));
    }
    AP::logger().Write("LTER", "R1,R2", "--", "--", "ZZ", r1, r2);

    // fields start after the "servingcell" token and its comma
    p = sc + strlen("\"servingcell\"");
    if (*p != ',') {
        return false;
    }
    p++;

    const char *f[24];
    uint16_t l[24];
    const uint8_t n = csv_split(p, f, l, 24);
    for (uint8_t i = 0; i < n; i++) {
        unquote(f[i], l[i]);
    }

    // f[0] is the <state> field ("NOCONN" and friends); the RAT is f[1]. These
    // are the applet's t[] indices less one, t[1] being the first field after
    // the "servingcell" token.
    if (n < 3) {
        return false;
    }
    const bool is_lte = (l[1] == 3 && memcmp(f[1], "LTE", 3) == 0) ||
                        (l[1] == 4 && memcmp(f[1], "eMTC", 4) == 0);
    if (!is_lte) {
        return false;
    }
    const bool is_fdd = (l[2] == 3 && memcmp(f[2], "FDD", 3) == 0);
    const uint8_t tac_idx = is_fdd ? 11 : 10;   // t[12]/t[11] in the applet
    if (n < tac_idx + 5) {
        return false;
    }

    const int32_t mcc    = dec_or(f[3], l[3], 0);
    const int32_t mnc    = dec_or(f[4], l[4], 0);
    const int32_t cid    = hex_or(f[5], l[5], 0);
    const int32_t pcid   = dec_or(f[6], l[6], 0);
    const int32_t earfcn = dec_or(f[7], l[7], 0);
    const int32_t band   = dec_or(f[8], l[8], -1);
    const int32_t tac    = hex_or(f[tac_idx], l[tac_idx], 0);
    const int32_t rsrp   = dec_or(f[tac_idx + 1], l[tac_idx + 1], 0);
    const int32_t rsrq   = dec_or(f[tac_idx + 2], l[tac_idx + 2], 0);
    const int32_t rssi   = dec_or(f[tac_idx + 3], l[tac_idx + 3], 0);
    const int32_t sinr   = dec_or(f[tac_idx + 4], l[tac_idx + 4], 0);

    AP::logger().Write("LTES", "MCC,MNC,TAC,CID,PID,EF,RSRP,RSRQ,RSSI,SINR",
                       "iiiiiiiiii",
                       mcc, mnc, tac, cid, pcid, earfcn, rsrp, rsrq, rssi, sinr);

    const int32_t tower_id = cid >> 8;
    if (_track_band < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE: connected on Band %ld (EARFCN %ld)",
                      long(band), long(earfcn));
        _track_band = band;
    } else if (band != _track_band) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE WARNING: band switch %ld -> %ld (EARFCN %ld)",
                      long(_track_band), long(band), long(earfcn));
        _track_band = band;
    }
    if (_track_cid < 0) {
        _track_cid = tower_id;
    } else if (tower_id != _track_cid) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE WARNING: cell tower switch CID %ld -> %ld",
                      long(_track_cid), long(tower_id));
        _track_cid = tower_id;
    }
    return true;
}

bool AP_LTE::check_QCSQ()
{
    const char *p = rx_at("+QCSQ:");
    if (p == nullptr) {
        return false;
    }
    // +QCSQ: "<sysmode>",<rssi>,<rsrp>,<sinr>,<rsrq>
    p += strlen("+QCSQ:");
    int32_t rssi = 0, rsrp = 0, sinr_raw = 0, rsrq = 0;
    if (!skip_quoted(p) ||
        !parse_int_after(p, ',', rssi) ||
        !parse_int_after(p, ',', rsrp) ||
        !parse_int_after(p, ',', sinr_raw) ||
        !parse_int_after(p, ',', rsrq)) {
        return false;
    }
    AP::logger().Write("LTEQ", "RSSI,RSRP,SINRr,RSRQ", "iiii",
                       rssi, rsrp, sinr_raw, rsrq);
    gcs().send_named_float("LTE_RSSI", rssi);
    gcs().send_named_float("LTE_RSRP", rsrp);
    gcs().send_named_float("LTE_RSRQ", rsrq);
    // QCSQ <sinr> is a scaled index, not dB, so it is deliberately not
    // published as LTE_SINR -- the slow QENG poll owns that with a real dB
    // value. It is logged above as SINRr.
    return true;
}

void AP_LTE::handle_AT_reply()
{
    check_CSQ();
    if (check_CPSI()) {
        return;
    }
    if (check_QENG()) {
        return;
    }
    check_QCSQ();
}

// ---------------------------------------------------------------------------
// identification
// ---------------------------------------------------------------------------

/*
  Firmware revision out of the ATI reply, which looks like
    Quectel\r\nEC25\r\nRevision: EC25EFAR08A07M4G\r\nOK\r\n
 */
void AP_LTE::check_modem_revision(const char *s)
{
    const char *rev = nullptr;
    uint16_t rev_len = 0;

    const char *p = strstr(s, "Revision:");
    if (p != nullptr) {
        p += strlen("Revision:");
        while (*p == ' ') {
            p++;
        }
        const char *e = p;
        while ((*e >= '0' && *e <= '9') || (*e >= 'A' && *e <= 'Z') ||
               (*e >= 'a' && *e <= 'z') || *e == '.' || *e == '_') {
            e++;
        }
        rev = p;
        rev_len = uint16_t(e - p);
    } else {
        // fall back to a bare family string
        static const char *const prefixes[] = { "EC25", "EC20", "BG95" };
        for (uint8_t i = 0; i < ARRAY_SIZE(prefixes); i++) {
            const char *q = strstr(s, prefixes[i]);
            if (q == nullptr) {
                continue;
            }
            const char *e = q;
            while ((*e >= '0' && *e <= '9') || (*e >= 'A' && *e <= 'Z') ||
                   (*e >= 'a' && *e <= 'z')) {
                e++;
            }
            rev = q;
            rev_len = uint16_t(e - q);
            break;
        }
    }

    if (rev == nullptr || rev_len == 0) {
        return;
    }
    copy_field(_modem_revision, sizeof(_modem_revision), rev, rev_len);

    // Short form for the identity block and the LTEI log. Quectel buries the
    // useful part as R<nn>A<nn> ("EC25EFAR08A07M4G" -> "R08A07"); SimCom puts
    // it after an underscore ("SIM7600G_V2.0.2" -> "V2.0.2"). Anything
    // matching neither keeps the full string.
    _fw_short[0] = 0;
    for (const char *q = _modem_revision; q[0] != 0 && q[1] != 0; q++) {
        if (q[0] == 'R' && isdigit(q[1]) && isdigit(q[2]) &&
            (q[3] == 'A' || q[3] == 'a') && isdigit(q[4]) && isdigit(q[5])) {
            copy_field(_fw_short, sizeof(_fw_short), q, 6);
            break;
        }
    }
    if (_fw_short[0] == 0) {
        for (const char *q = _modem_revision; q[0] != 0; q++) {
            if (q[0] == 'V' && isdigit(q[1])) {
                const char *e = q + 1;
                while (isdigit(*e) || *e == '.') {
                    e++;
                }
                while (e > q && e[-1] == '.') {
                    e--;
                }
                copy_field(_fw_short, sizeof(_fw_short), q, uint16_t(e - q));
                break;
            }
        }
    }
    if (_fw_short[0] == 0) {
        strncpy(_fw_short, _modem_revision, sizeof(_fw_short) - 1);
        _fw_short[sizeof(_fw_short) - 1] = 0;
    }
}

void AP_LTE::check_modem_banner()
{
    for (uint8_t i = 0; i < AP_LTE_modem_count; i++) {
        const AP_LTE_ModemInfo &m = AP_LTE_modem_list[i];
        if (m.banner == nullptr || !rx_find(m.banner)) {
            continue;
        }
        _modem = &m;
        // Not announced here: step_SIMINFO prints model and firmware once.
        check_modem_revision(_rx);
        return;
    }
}

// ---------------------------------------------------------------------------
// auth response
// ---------------------------------------------------------------------------

// first dotted quad in the body, returned with the pointer just past it
static const char *find_dotted_quad(const char *s, uint8_t ip[4])
{
    for (const char *p = s; *p != 0; p++) {
        if (!isdigit(*p)) {
            continue;
        }
        // only try at the start of a number, so a failed match on "10.9.8"
        // does not then succeed one digit later
        if (p != s && (isdigit(p[-1]) || p[-1] == '.')) {
            continue;
        }
        const char *q = p;
        if (AP_LTE::parse_ipv4(q, ip)) {
            return q;
        }
    }
    return nullptr;
}

// Handles all three shapes the endpoint may answer with: JSON, ip:port, and
// ip followed by port on the next line.
bool AP_LTE::parse_auth_response(const char *body, uint8_t ip[4], uint16_t &port) const
{
    const char *jip = strstr(body, "\"ip\"");
    const char *jpt = strstr(body, "\"port\"");
    if (jip != nullptr && jpt != nullptr) {
        // the next quote past the key is the opening quote of the value
        const char *q = strchr(jip + 4, '"');
        const char *c = strchr(jpt + 6, ':');
        int32_t p = 0;
        if (q != nullptr && c != nullptr) {
            const char *vp = q + 1;
            const char *pp = c + 1;
            if (parse_ipv4(vp, ip) && parse_int(pp, p) && p > 0 && p <= 65535) {
                port = uint16_t(p);
                return true;
            }
        }
    }

    const char *after = find_dotted_quad(body, ip);
    if (after == nullptr) {
        return false;
    }
    while (*after != 0 && !isdigit(*after)) {
        after++;
    }
    const long p = strtol(after, nullptr, 10);
    if (p <= 0 || p > 65535) {
        return false;
    }
    port = uint16_t(p);
    return true;
}

// 1970-01-01 -> 2015-01-01, matching the epoch_offset in
// GCS_MAVLINK::update_signing_timestamp().
static const uint64_t SIGNING_EPOCH_OFFSET_S = 1420070400ULL;

/*
  Convert the auth server's millisecond Unix epoch into MAVLink2 signing
  format, 10us ticks since 2015-01-01. Seeding a real "now" rather than zero
  stops the receiving GCS rejecting the first packets as stale, and stops the
  persisted signing timestamp being wound backwards.
 */
static bool signing_timestamp_from_epoch_ms(const char *digits, uint64_t &out)
{
    if (!isdigit(*digits)) {
        return false;
    }
    uint64_t ms = 0;
    for (const char *p = digits; isdigit(*p); p++) {
        ms = ms * 10 + uint64_t(*p - '0');
    }
    uint64_t secs = ms / 1000;
    if (secs > SIGNING_EPOCH_OFFSET_S) {
        secs -= SIGNING_EPOCH_OFFSET_S;
    }
    out = secs * 100000ULL;
    return true;
}

static bool hex_decode(const char *hexstr, uint8_t *out, uint8_t expected_len)
{
    for (uint8_t i = 0; i < expected_len; i++) {
        const int hi = hex_or(&hexstr[i * 2], 1, -1);
        const int lo = hex_or(&hexstr[i * 2 + 1], 1, -1);
        if (hi < 0 || lo < 0) {
            return false;
        }
        out[i] = uint8_t((hi << 4) | lo);
    }
    return true;
}

/*
  Pull sysId and mavlinkSigningKey out of the same body the address came from
  and apply them.
 */
void AP_LTE::apply_auth_identity(const char *body)
{
    if (pget(_authsys) == 1) {
        const char *p = strstr(body, "\"sysId\"");
        if (p != nullptr) {
            const char *c = strchr(p, ':');
            const long sysid = (c != nullptr) ? strtol(c + 1, nullptr, 10) : 0;
            if (sysid > 0 && sysid < 256) {
                // set_and_save, not set: the next boot then starts on the id
                // this server granted rather than switching only once HTTPAUTH
                // lands 15-20s in.
                if (!AP_Param::set_and_save_by_name("SYSID_THISMAV", float(sysid))) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: sysid save failed");
                }
                // mavlink_system.sysid is latched from the parameter once at
                // boot and never re-read, so the live value needs setting too.
                if (!hal.util->get_soft_armed()) {
                    mavlink_system.sysid = uint8_t(sysid);
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: sysid set to %ld", sysid);
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: sysid set failed (armed?)");
                }
            } else if (sysid == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "LTE HTTPAUTH: ignoring invalid sysid 0 from server");
            }
        }
    }

    if (pget(_authkey) != 1) {
        return;
    }

    const char *k = strstr(body, "\"mavlinkSigningKey\"");
    if (k == nullptr) {
        return;
    }
    const char *q = strchr(k + strlen("\"mavlinkSigningKey\""), '"');
    if (q == nullptr) {
        return;
    }
    q++;
    const char *e = strchr(q, '"');
    if (e == nullptr) {
        return;
    }
    if (e - q != 64) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: signing key wrong length");
        return;
    }
    uint8_t key[32];
    if (!hex_decode(q, key, 32)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: signing key wrong length");
        return;
    }

    uint64_t initial_ts = 0;
    bool have_ts = false;
    const char *t = strstr(body, "\"timestamp\"");
    if (t != nullptr) {
        const char *c = strchr(t, ':');
        if (c != nullptr) {
            c++;
            while (*c == ' ' || *c == '"') {
                c++;
            }
            have_ts = signing_timestamp_from_epoch_ms(c, initial_ts);
        }
    }

    if (!have_ts) {
        // Do not fall back to zero: set_signing_key() unconditionally
        // overwrites the persisted signing timestamp, and seeding it near the
        // 2015 epoch would wind it backwards, reopening a replay window for
        // anything captured under this key since. Skip this round instead.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "LTE HTTPAUTH: no server timestamp, signing key not applied");
    } else if (gcs().set_signing_key(key, initial_ts)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LTE HTTPAUTH: signing key applied");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTE HTTPAUTH: signing key apply failed (armed?)");
    }
}

#endif  // AP_LTE_ENABLED
