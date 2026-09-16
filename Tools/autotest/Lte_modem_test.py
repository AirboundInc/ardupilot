#!/usr/bin/env python3
"""
LTE Modem SITL Happy-Path Test
-------------------------------
1. Runs a mock EC25 modem on a PTY — answers the AT chain, including the
   HTTPAUTH step's Quectel HTTP(S) commands, and validates QIOPEN
2. Connects to MAVLink, waits for heartbeat
3. Background thread injects LTE_SERVER_* params via PARAM_SET
4. Main thread monitors STATUSTEXT from the start (no missed steps)
5. Validates step sequence, the address HTTPAUTH returned, and QIOPEN

Exit codes:  0 = all checks pass,  1 = timeout / error / wrong sequence
"""
import re
import sys
import time
import serial
import threading
import argparse
from pymavlink import mavutil

# ── Server destination handed back by the mock's HTTPS auth response ──
EXPECTED_IP = "8.8.8.8"
EXPECTED_PORT = 8080

# ── Static fallback injected as LTE_SERVER_* params ───────────────────
# Deliberately different from the address above: HTTPAUTH is supposed to
# override the static params, so a QIOPEN to this address means
# effective_server() picked the fallback and the auth result was lost.
STATIC_IP = "10.9.8.7"
STATIC_PORT = 9999

# ── Expected step order for the happy path ────────────────────────────
# Allow CIPCLOSE between SOCKET_STATE and final CIPOPEN/CONNECTED
ALLOWED_STEPS = {"ATI", "CMUX", "BAUD", "CPIN", "CONFIG", "CREG", "HTTPAUTH",
                 "SIGNAL_GATE", "QENG", "SOCKET_STATE", "CIPOPEN",
                 "CIPCLOSE", "CONNECTED"}
# ATI is not required even though it always runs first. AP_LTE starts from
# init_ardupilot(), before any GCS link exists, so its "step ATI" statustext is
# emitted into a void and dropped. Reaching BAUD proves ATI completed anyway:
# that is the only way out of ATI, and it needs the modem banner parsed.
REQUIRED_STEPS = ["CPIN", "CREG", "HTTPAUTH", "CIPOPEN", "CONNECTED"]
# Without HTTPAUTH the driver takes the static LTE_SERVER_* address instead.
REQUIRED_STEPS_NO_AUTH = ["CPIN", "CREG", "CIPOPEN", "CONNECTED"]

# Set from --no-httpauth; see the argument's help text for why it exists.
NO_HTTPAUTH = False

# ── AT command → response table (happy-path) ──────────────────────────
AT_RESPONSES = {
    'ATI':        b'\r\nEC25EFAR06A18M4G\r\nOK\r\n',
    'AT+CPIN?':   b'\r\n+CPIN: READY\r\nOK\r\n',
    'AT+CEREG?':  b'\r\n+CEREG: 0,1\r\nOK\r\n',
    'AT+CREG?':   b'\r\n+CREG: 0,1\r\nOK\r\n',
    'AT+QCSQ':    b'\r\n+QCSQ: "eMTC",-65,-10,18,-8\r\nOK\r\n',
    'AT+CPSI?':   b'\r\n+CPSI: LTE,Online,404-45,0x0001,1A2B3C,300,'
                  b'EUTRAN-BAND1,1,3,3,-110,-12,-80,11\r\nOK\r\n',
    # Real EC25 shape, including the <state> field. It used to be omitted,
    # which made every field after it shift by one and the reply parse as
    # not-LTE, so the servingcell parser was never actually exercised.
    # "servingcell",<state>,<rat>,<is_tdd>,<MCC>,<MNC>,<cellID>,<PCID>,
    # <earfcn>,<band>,<ULbw>,<DLbw>,<TAC>,<RSRP>,<RSRQ>,<RSSI>,<SINR>
    'AT+QENG=':   b'\r\n+QENG: "servingcell","NOCONN","LTE","FDD",404,45,'
                  b'1A2B3C,300,1650,3,5,5,3A5F,-88,-10,-60,14,-\r\nOK\r\n',
    'AT+QISTATE': b'\r\nOK\r\n',
    'AT+QICLOSE': b'\r\nOK\r\n',
    'AT+COPS':    b'\r\nOK\r\n',
}

# ── HTTPAUTH (HTTPS) support ──────────────────────────────────────────
# step_HTTPAUTH() POSTs over TLS to fetch the real server address, so the
# mock has to answer the Quectel HTTP(S) chain. AT+QFLST reports the CA
# cert as already present, which skips the ~1.7KB AT+QFUPL upload — that
# is also what a provisioned module does on every boot after the first.
CA_CERT_FILENAME = "isrgrootx1.pem"

# Body returned by AT+QHTTPREAD; parse_auth_response() reads "ip"/"port"
# from it. sysId and mavlinkSigningKey are deliberately absent — granting
# them would move SYSID_THISMAV and switch on MAVLink signing mid-test.
AUTH_RESPONSE_JSON = (
    f'{{"ip":"{EXPECTED_IP}","port":{EXPECTED_PORT}}}'.encode())

# ── Params to inject after LTE driver registers them ──────────────────
INJECTED_PARAMS = {
    'LTE_SERVER_PORT': STATIC_PORT,
    'LTE_SERVER_IP0':  int(STATIC_IP.split('.')[0]),
    'LTE_SERVER_IP1':  int(STATIC_IP.split('.')[1]),
    'LTE_SERVER_IP2':  int(STATIC_IP.split('.')[2]),
    'LTE_SERVER_IP3':  int(STATIC_IP.split('.')[3]),
}

# ── Shared state for cross-thread QIOPEN validation ──────────────────
qiopen_result = {
    'received': False,
    'validated': False,
    'command': '',
}
qiopen_lock = threading.Lock()

# The live CmuxLink, so the checks at the end can ask what actually reached
# the socket channel.
mock_link = None


def handle_qiopen(line_str, ser):
    """Validate QIOPEN has correct IP and port, reject if wrong."""
    expected_fragment = f'"{EXPECTED_IP}",{EXPECTED_PORT}'

    with qiopen_lock:
        qiopen_result['received'] = True
        qiopen_result['command'] = line_str

        if expected_fragment in line_str:
            qiopen_result['validated'] = True
            print(f"[MOCK] TX: QIOPEN validated OK "
                  f"({EXPECTED_IP}:{EXPECTED_PORT})", flush=True)
            ser.write(b'\r\nOK\r\n+QIOPEN: 0,0\r\n')
            # Access mode 2: the channel that opened the socket now carries
            # it. Unmuxed that is the whole port; muxed it is DLC 2 -- and if
            # the driver opened on DLC 1, DLC 2 never goes transparent and
            # nothing it writes there is ever counted.
            if hasattr(ser, 'data_transparent') and \
                    (not ser.muxed or ser.reply_dlc == 2):
                ser.data_transparent = True
        else:
            qiopen_result['validated'] = False
            print(f"[MOCK] TX: QIOPEN REJECTED — expected "
                  f"{expected_fragment}, got: {line_str}", flush=True)
            ser.write(b'\r\nOK\r\n+QIOPEN: 0,1\r\n')

    ser.flush()


def finish_upload(pending, ser):
    """Answer the terminator-less payload written after a CONNECT prompt."""
    if pending['kind'] == 'url':
        ser.write(b'\r\nOK\r\n')
        print(f"[MOCK] TX: QHTTPURL OK "
              f"({pending['got'].decode('utf-8', 'ignore')})", flush=True)
    else:
        ser.write(b'\r\nOK\r\n\r\n+QHTTPPOST: 0,200,%d\r\n'
                  % len(AUTH_RESPONSE_JSON))
        print(f"[MOCK] TX: QHTTPPOST 0,200 (body {len(pending['got'])}B)",
              flush=True)
    ser.flush()


def _fcs_table():
    tbl = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xE0 if c & 1 else c >> 1
        tbl.append(c)
    return tbl


CMUX_FCS = _fcs_table()
CMUX_FLAG = 0xF9


def cmux_fcs(header):
    fcs = 0xFF
    for b in header:
        fcs = CMUX_FCS[fcs ^ b]
    return (~fcs) & 0xFF


def cmux_frame(dlc, payload, ctrl=0xEF):
    """One GSM 07.10 basic-mode frame. ctrl 0xEF is UIH, 0x73 is UA."""
    hdr = bytes([(dlc << 2) | 0x01 | 0x02, ctrl | 0x10])
    n = len(payload)
    hdr += bytes([(n << 1) | 0x01]) if n <= 127 \
        else bytes([(n & 0x7F) << 1, (n >> 7) & 0xFF])
    return bytes([CMUX_FLAG]) + hdr + payload + \
        bytes([cmux_fcs(hdr), CMUX_FLAG])


class CmuxLink:
    """Serial port that can speak either plain AT or GSM 07.10 basic mode.

    The AT logic above is written against a plain port, so the framing is
    hidden here: read() hands back AT payload and write() frames the reply
    onto whichever channel the command arrived on.

    Both DLC 1 and DLC 2 are AT channels to start with, as on a real modem.
    Quectel's access mode 2 binds the socket to the channel that issued the
    open, so once QIOPEN succeeds DLC 2 goes transparent and everything on it
    is counted as socket bytes. A driver that opens the socket on DLC 1 and
    then writes telemetry to DLC 2 gets no CONNECT and no data through.
    """

    def __init__(self, ser):
        self.ser = ser
        self.muxed = False
        self.partial = b''
        self.data_rx = 0
        self.data_buf = b''
        self.sabm_seen = 0
        self.reply_dlc = 1
        self.data_transparent = False

    def read(self, n):
        data = self.ser.read(n)
        if not data:
            return data
        if not self.muxed:
            # Unmuxed there is one stream, so once the socket is open its
            # bytes and the driver's AT polling are the same bytes. Count
            # them, and still hand them to the AT parser.
            if self.data_transparent:
                self.data_rx += len(data)
                self.data_buf = (self.data_buf + data)[-8192:]
            return data
        self.partial += data
        return self._deframe()

    def _deframe(self):
        at = b''
        buf = self.partial
        pos = 0
        while pos < len(buf):
            while pos < len(buf) and buf[pos] != CMUX_FLAG:
                pos += 1
            if len(buf) - pos < 6:
                break
            addr, ctrl, l1 = buf[pos + 1], buf[pos + 2], buf[pos + 3]
            if l1 & 0x01:
                plen, hlen = l1 >> 1, 3
            else:
                if len(buf) - pos < 7:
                    break
                plen, hlen = (l1 >> 1) | (buf[pos + 4] << 7), 4
            flen = 1 + hlen + plen + 2
            if len(buf) - pos < flen:
                break
            if buf[pos + flen - 1] != CMUX_FLAG:
                pos += 1
                continue
            dlc = (addr >> 2) & 0x3F
            payload = buf[pos + 1 + hlen:pos + 1 + hlen + plen]
            if (ctrl & 0xEF) == 0xEF:
                if dlc == 2 and self.data_transparent:
                    self.data_rx += len(payload)
                    self.data_buf = (self.data_buf + payload)[-8192:]
                elif dlc in (1, 2):
                    at += payload
                    self.reply_dlc = dlc
            elif (ctrl & ~0x10) == 0x2F:          # SABM: acknowledge with UA
                self.sabm_seen += 1
                self.ser.write(cmux_frame(dlc, b'', ctrl=0x63))
            pos += flen
        self.partial = buf[pos:]
        return at

    def write(self, data):
        self.ser.write(cmux_frame(self.reply_dlc, data) if self.muxed else data)

    def inject_socket(self, data):
        """Push bytes into the open socket, the way the relay server would.

        Muxed this has to be DLC 2, the channel the socket was opened on.
        This is the GCS -> vehicle direction, which nothing else here covers:
        every other check only proves the vehicle can transmit.
        """
        self.ser.write(cmux_frame(2, data) if self.muxed else data)
        self.ser.flush()

    def flush(self):
        self.ser.flush()


def handle_line(line_str, ser):
    """Respond to one AT command.

    Returns a pending-upload dict when the reply was a CONNECT prompt, so
    the caller stops line-splitting and counts raw bytes instead.
    """
    if 'AT+CMUX' in line_str:
        time.sleep(0.05)
        # Echoed command then OK, because ATE0 does not go out until CONFIG.
        # The port only starts framing after this reply is on the wire.
        ser.ser.write(b'AT+CMUX=0\r\r\nOK\r\n')
        ser.ser.flush()
        ser.muxed = True
        print("[MOCK] TX: CMUX=0 OK, switching to framed mode", flush=True)
        return None

    if 'AT+QIOPEN' in line_str:
        time.sleep(0.05)
        handle_qiopen(line_str, ser)
        return None

    # --- HTTPAUTH: Quectel HTTP(S) chain ---
    if 'AT+QFLST' in line_str:
        time.sleep(0.05)
        ser.write(b'\r\n+QFLST: "%s",1391\r\n\r\nOK\r\n'
                  % CA_CERT_FILENAME.encode())
        ser.flush()
        print("[MOCK] TX: QFLST (cert present, upload skipped)", flush=True)
        return None

    # QHTTPURL/QHTTPPOST answer CONNECT, then take exactly <len> raw bytes
    # with no CR/LF — they cannot go through the line splitter below.
    for cmd, kind in (('AT+QHTTPURL=', 'url'), ('AT+QHTTPPOST=', 'post')):
        if cmd not in line_str:
            continue
        m = re.search(re.escape(cmd) + r'(\d+)', line_str)
        if not m:
            continue                       # malformed: fall through to generic OK
        need = int(m.group(1))
        time.sleep(0.05)
        ser.write(b'\r\nCONNECT\r\n')
        ser.flush()
        print(f"[MOCK] TX: CONNECT ({kind}, expecting {need}B)", flush=True)
        return {'kind': kind, 'need': need, 'got': b''}

    if 'AT+QHTTPREAD' in line_str:
        time.sleep(0.05)
        ser.write(b'\r\nCONNECT\r\n' + AUTH_RESPONSE_JSON +
                  b'\r\nOK\r\n\r\n+QHTTPREAD: 0\r\n')
        ser.flush()
        print(f"[MOCK] TX: QHTTPREAD -> "
              f"{AUTH_RESPONSE_JSON.decode()}", flush=True)
        return None

    for key, resp in AT_RESPONSES.items():
        if key in line_str:
            time.sleep(0.05)
            ser.write(resp)
            ser.flush()
            print(f"[MOCK] TX: {key}", flush=True)
            return None

    # QHTTPCFG / QSSLCFG / QIACT and friends only need a bare OK
    if 'AT' in line_str:
        time.sleep(0.05)
        ser.write(b'\r\nOK\r\n')
        ser.flush()
        print("[MOCK] TX: Generic OK", flush=True)
    return None


def check_uplink(link):
    """Send a param read into the socket, wait for the answer to come back.

    Proves the GCS -> vehicle direction end to end. A vehicle that can
    transmit but never receives looks perfectly healthy until you try to
    command it.
    """
    if link is None:
        print("\n⚠️  WARN: no mock link, uplink not checked", flush=True)
        return True

    from pymavlink.dialects.v20 import ardupilotmega as mav2

    mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
    req = mav.param_request_read_encode(1, 1, b"SYSID_THISMAV", -1).pack(mav)

    # Matched on the raw bytes rather than decoded: unmuxed the socket stream
    # carries the driver's own AT polling alongside telemetry, so a strict
    # parser trips over it. PARAM_VALUE carries param_id as plain ASCII, and
    # the request travels the other way, so this can only be the reply.
    needle = b"SYSID_THISMAV"
    link.data_buf = b''
    deadline = time.time() + 15
    next_send = 0.0
    while time.time() < deadline:
        if time.time() >= next_send:
            link.inject_socket(req)
            next_send = time.time() + 1.0
            print("[MOCK] TX->socket: PARAM_REQUEST_READ SYSID_THISMAV",
                  flush=True)
        if needle in link.data_buf:
            print("✅ GCS->vehicle round trip OK: PARAM_VALUE came back "
                  "through the socket", flush=True)
            return True
        time.sleep(0.2)

    print("\n❌ FAIL: no answer to PARAM_REQUEST_READ sent into the socket",
          flush=True)
    print(f"   The vehicle transmits ({link.data_rx}B out) but is not acting "
          f"on what it receives.", flush=True)
    if link.muxed:
        print("   Multiplexed: check the driver reads socket bytes from the "
              "same DLC it opened the socket on.", flush=True)
    return False


def mock_modem_thread(port):
    """Serial listener that responds to AT commands."""
    global mock_link
    try:
        ser = CmuxLink(serial.Serial(port, 115200, timeout=0.1))
        mock_link = ser
        print(f"[MOCK] Listening on {port}...", flush=True)
        buf = b''
        pending = None          # set while a CONNECT payload is inbound
        last_rx = time.time()
        while True:
            data = ser.read(256)
            if data:
                buf += data
                last_rx = time.time()

            if pending is not None:
                want = pending['need'] - len(pending['got'])
                if want > 0 and buf:
                    pending['got'] += buf[:want]
                    buf = buf[want:]
                complete = len(pending['got']) >= pending['need']
                if (not complete and pending['got']
                        and time.time() - last_rx > 1.0):
                    # Answer a short payload rather than wedging the step
                    # until its 25s timeout; a dropped byte on the PTY
                    # should not be reported as a script failure.
                    print(f"[MOCK] WARN: {pending['kind']} upload short "
                          f"({len(pending['got'])}/{pending['need']}B)",
                          flush=True)
                    complete = True
                if not complete:
                    continue
                finish_upload(pending, ser)
                pending = None

            while (b'\r' in buf or b'\n' in buf) and pending is None:
                idx_r = buf.find(b'\r')
                idx_n = buf.find(b'\n')
                if idx_r != -1 and idx_n != -1:
                    idx = min(idx_r, idx_n)
                else:
                    idx = max(idx_r, idx_n)

                line = buf[:idx].strip()
                term = buf[idx:idx + 1]
                buf = buf[idx + 1:]
                # Eat the companion byte of a CRLF/LFCR pair. Leaving it is
                # harmless while we keep line-splitting, but a CONNECT
                # payload begins immediately after this and would take the
                # stray byte as its first, dropping a real one off the end.
                if buf[:1] in (b'\r', b'\n') and buf[:1] != term:
                    buf = buf[1:]

                if not line:
                    continue

                line_str = line.decode('utf-8', errors='ignore')
                print(f"[MOCK] RX: {line_str}", flush=True)
                pending = handle_line(line_str, ser)

    except Exception as e:
        print(f"[MOCK] Serial error: {e}", flush=True)


def inject_params_thread(mavlink_port):
    """Background thread: connect separately and inject params."""
    # Separate MAVLink connection so we don't steal STATUSTEXT from main
    print("[INJECT] Connecting to MAVLink for param injection...",
          flush=True)
    conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{mavlink_port}')
    conn.wait_heartbeat()
    print(f"[INJECT] Heartbeat received, waiting 3s for LTE driver...",
          flush=True)
    time.sleep(3)

    print("[INJECT] Sending PARAM_SET messages...", flush=True)
    for name, value in INJECTED_PARAMS.items():
        name_bytes = name.encode('utf-8')

        acked = False
        for attempt in range(5):
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                name_bytes,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            # Drain messages looking for our specific ACK
            deadline = time.time() + 1.0
            while time.time() < deadline:
                msg = conn.recv_match(type='PARAM_VALUE', blocking=True,
                                      timeout=0.5)
                if msg and msg.param_id.strip('\x00') == name:
                    print(f"  [INJECT] {name} = {msg.param_value}",
                          flush=True)
                    acked = True
                    break
            if acked:
                break
            time.sleep(0.1)

        if not acked:
            print(f"  [INJECT] {name} — no ACK after 5 attempts "
                  f"(will rely on retry)", flush=True)

        time.sleep(0.05)

    print("[INJECT] Param injection complete.", flush=True)


def validate_steps(observed):
    """Check observed step sequence contains all required steps and uses only allowed steps."""
    # Check 1: all required steps must appear in order
    required_steps = REQUIRED_STEPS_NO_AUTH if NO_HTTPAUTH else REQUIRED_STEPS
    obs_idx = 0
    for required in required_steps:
        try:
            obs_idx = observed.index(required, obs_idx) + 1
        except ValueError:
            print(f"\n❌ FAIL: Required step '{required}' missing or out of order", flush=True)
            print(f"   Required (in order): {' → '.join(required_steps)}", flush=True)
            print(f"   Observed:            {' → '.join(observed)}", flush=True)
            return False

    # Check 2: no unexpected steps
    unexpected = [s for s in observed if s not in ALLOWED_STEPS]
    if unexpected:
        print(f"\n❌ FAIL: Unexpected steps in sequence: {unexpected}", flush=True)
        print(f"   Allowed: {sorted(ALLOWED_STEPS)}", flush=True)
        print(f"   Observed: {' → '.join(observed)}", flush=True)
        return False

    print(f"\n✅ Step sequence OK: {' → '.join(observed)}", flush=True)
    return True


def monitor_gcs(mavlink_port, inject_port, timeout):
    """Main thread: monitor STATUSTEXT from the very start."""
    print(f"[GCS] Waiting for MAVLink heartbeat on UDP {mavlink_port}...",
          flush=True)
    master = mavutil.mavlink_connection(f'udpin:0.0.0.0:{mavlink_port}')
    master.wait_heartbeat()
    print(f"[GCS] Heartbeat received (system {master.target_system},"
          f" component {master.target_component})", flush=True)

    # Start param injection in background — doesn't block monitoring.
    # Skipped without HTTPAUTH: the injected LTE_SERVER_* exist only to be
    # overridden by the auth result, so injecting them with auth off would
    # point the driver away from the address the mock accepts.
    if not NO_HTTPAUTH:
        inject_thread = threading.Thread(
            target=inject_params_thread, args=(inject_port,), daemon=True)
        inject_thread.start()

    # Monitor STATUSTEXT immediately — catch every step from ATI onwards
    print(f"[GCS] Monitoring STATUSTEXT for {timeout}s...", flush=True)
    start_time = time.time()
    observed_steps = []
    auth_server = None

    while time.time() - start_time < timeout:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
        if not msg:
            continue

        text = msg.text
        if "LTE" not in text:
            continue

        elapsed = int(time.time() - start_time)
        print(f"  [{elapsed:3d}s] {text}", flush=True)

        # Address handed back by the HTTPS auth response
        if "HTTPAUTH: server " in text:
            auth_server = text.split("HTTPAUTH: server ")[-1].strip()

        # Collect step transitions
        if "step " in text.lower():
            step_name = text.split("step ")[-1].strip()
            observed_steps.append(step_name)

        # The socket-open message is the final state. Matched exactly, not by
        # searching for "connected": "LTE: connected on Band 3 (EARFCN 1650)"
        # is a signal report that arrives well before the socket exists, and a
        # loose match reads it as success and stops the test early.
        elif text.strip() == "LTE_modem: connected":
            observed_steps.append("CONNECTED")

            # --- CHECK 1: Step sequence ---
            if not validate_steps(observed_steps):
                return 1

            # --- CHECK 2: HTTPAUTH returned the expected address ---
            expected_server = f"{EXPECTED_IP}:{EXPECTED_PORT}"
            if NO_HTTPAUTH:
                print("⏭  HTTPAUTH check skipped (--no-httpauth)", flush=True)
            elif auth_server == expected_server:
                print(f"✅ HTTPAUTH address OK: {auth_server}", flush=True)
            elif auth_server is None:
                print(f"\n❌ FAIL: HTTPAUTH never reported a server address",
                      flush=True)
                return 1
            else:
                print(f"\n❌ FAIL: HTTPAUTH returned {auth_server}, "
                      f"expected {expected_server}", flush=True)
                return 1

            # --- CHECK 3: QIOPEN destination ---
            # Must be the HTTPAUTH address, not the LTE_SERVER_* fallback.
            with qiopen_lock:
                if qiopen_result['received'] and qiopen_result['validated']:
                    print(f"✅ QIOPEN destination OK: "
                          f"{EXPECTED_IP}:{EXPECTED_PORT}", flush=True)
                elif qiopen_result['received']:
                    print(f"\n❌ FAIL: Connected to WRONG address!",
                          flush=True)
                    print(f"   QIOPEN cmd: {qiopen_result['command']}",
                          flush=True)
                    if STATIC_IP in qiopen_result['command']:
                        print(f"   Used the LTE_SERVER_* fallback — the "
                              f"HTTPAUTH address was not applied.", flush=True)
                    return 1
                else:
                    print(f"\n⚠️  WARN: CONNECTED but no QIOPEN seen",
                          flush=True)

            # --- CHECK 4: telemetry actually reaches the socket ---
            # Reaching CONNECTED only proves the socket opened. This proves
            # MAVLink is flowing through it -- and when multiplexed, that it
            # is on the channel the socket was opened on. Open the socket on
            # DLC 1 and write telemetry to DLC 2 and every check above still
            # passes while nothing leaves the vehicle.
            deadline = time.time() + 10
            while time.time() < deadline:
                if mock_link is not None and mock_link.data_rx > 0:
                    break
                time.sleep(0.2)
            if mock_link is None:
                print("\n⚠️  WARN: no mock link to inspect", flush=True)
            elif mock_link.data_rx == 0:
                print("\n❌ FAIL: CONNECTED but no bytes reached the socket "
                      "channel in 10s", flush=True)
                if mock_link.muxed:
                    print("   Multiplexed: the socket was opened on one DLC "
                          "and the telemetry written to another.", flush=True)
                return 1
            elif b'\xfd' not in mock_link.data_buf:
                print(f"\n❌ FAIL: {mock_link.data_rx}B reached the socket but "
                      f"none of it is MAVLink2 (no 0xFD)", flush=True)
                return 1
            else:
                print(f"✅ Socket carries MAVLink2: {mock_link.data_rx}B",
                      flush=True)

            # --- CHECK 5: GCS -> vehicle round trip ---
            # Everything above only proves the vehicle can transmit. This
            # pushes a PARAM_REQUEST_READ into the socket the way the relay
            # server would and waits for the PARAM_VALUE to come back out of
            # it, which exercises socket -> driver -> virtual port -> MAVLink
            # backend -> reply -> socket. Without it a driver that receives
            # nothing at all still passes every other check.
            if not check_uplink(mock_link):
                return 1

            print(f"\n✅ SUCCESS: All checks passed.", flush=True)
            return 0

        # Fatal errors — bail out instead of burning the rest of the timeout
        fail_keywords = ["could not find", "bad step", "halted", "halting"]
        if any(kw in text.lower() for kw in fail_keywords):
            print(f"\n❌ FAIL: LTE driver error — {text}", flush=True)
            if observed_steps:
                print(f"   Got as far as: {' → '.join(observed_steps)}",
                      flush=True)
            return 1

    print("\n❌ FAIL: Timed out waiting for CONNECTED state.", flush=True)
    if observed_steps:
        print(f"   Got as far as: {' → '.join(observed_steps)}", flush=True)
    return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Mock EC25 modem + MAVLink monitor for LTE SITL tests")
    parser.add_argument("--port", required=True,
                        help="PTY path for the mock modem")
    parser.add_argument("--mavlink-port", type=int, required=True,
                        help="UDP port for STATUSTEXT monitoring")
    parser.add_argument("--inject-port", type=int, default=0,
                        help="UDP port for param injection (default: mavlink-port + 1)")
    parser.add_argument("--timeout", type=int, default=120,
                        help="Seconds to wait for CONNECTED (default 120)")
    parser.add_argument("--no-httpauth", action="store_true",
                        help="Vehicle has LTE_HTTPAUTH=0 and a static "
                             "LTE_SERVER_* address. Skips param injection and "
                             "the auth assertions, and still checks the step "
                             "sequence and the QIOPEN destination. Needed "
                             "while CI cannot provision the craft ID and "
                             "password the HTTPS login requires.")
    args = parser.parse_args()

    NO_HTTPAUTH = args.no_httpauth
    inject_port = args.inject_port if args.inject_port else args.mavlink_port + 1

    threading.Thread(target=mock_modem_thread, args=(args.port,),
                     daemon=True).start()
    sys.exit(monitor_gcs(args.mavlink_port, inject_port, args.timeout))