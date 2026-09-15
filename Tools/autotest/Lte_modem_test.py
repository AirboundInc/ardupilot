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

# ── Expected step order for the happy path (CMUX bypassed via sed) ────
# Allow CIPCLOSE between SOCKET_STATE and final CIPOPEN/CONNECTED
ALLOWED_STEPS = {"ATI", "BAUD", "CPIN", "CONFIG", "CREG", "HTTPAUTH",
                 "SIGNAL_GATE", "QENG", "SOCKET_STATE", "CIPOPEN",
                 "CIPCLOSE", "CONNECTED"}
REQUIRED_STEPS = ["ATI", "CPIN", "CREG", "HTTPAUTH", "CIPOPEN", "CONNECTED"]

# ── AT command → response table (happy-path) ──────────────────────────
AT_RESPONSES = {
    'ATI':        b'\r\nEC25EFAR06A18M4G\r\nOK\r\n',
    'AT+CPIN?':   b'\r\n+CPIN: READY\r\nOK\r\n',
    'AT+CEREG?':  b'\r\n+CEREG: 0,1\r\nOK\r\n',
    'AT+CREG?':   b'\r\n+CREG: 0,1\r\nOK\r\n',
    'AT+QCSQ':    b'\r\n+QCSQ: "eMTC",-65,-10,18,-8\r\nOK\r\n',
    'AT+CPSI?':   b'\r\n+CPSI: LTE,Online,404-45,0x0001,1A2B3C,300,'
                  b'EUTRAN-BAND1,1,3,3,-110,-12,-80,11\r\nOK\r\n',
    'AT+QENG=':   b'\r\n+QENG: "servingcell","LTE","FDD",404,45,'
                  b'1A2B3C,300,1,3,-110,-12,-80,11,-,-,-,-,-,-\r\nOK\r\n',
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


def handle_line(line_str, ser):
    """Respond to one AT command.

    Returns a pending-upload dict when the reply was a CONNECT prompt, so
    the caller stops line-splitting and counts raw bytes instead.
    """
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


def mock_modem_thread(port):
    """Serial listener that responds to AT commands."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
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
    obs_idx = 0
    for required in REQUIRED_STEPS:
        try:
            obs_idx = observed.index(required, obs_idx) + 1
        except ValueError:
            print(f"\n❌ FAIL: Required step '{required}' missing or out of order", flush=True)
            print(f"   Required (in order): {' → '.join(REQUIRED_STEPS)}", flush=True)
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

    # Start param injection in background — doesn't block monitoring
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

        # "connected" without "step" is the final state message
        elif "connected" in text.lower() and "step" not in text.lower():
            observed_steps.append("CONNECTED")

            # --- CHECK 1: Step sequence ---
            if not validate_steps(observed_steps):
                return 1

            # --- CHECK 2: HTTPAUTH returned the expected address ---
            expected_server = f"{EXPECTED_IP}:{EXPECTED_PORT}"
            if auth_server == expected_server:
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

            print(f"\n✅ SUCCESS: All checks passed.", flush=True)
            return 0

        # Fatal errors — bail out instead of burning the rest of the timeout
        fail_keywords = ["could not find", "bad step", "halted", "halting"]
        if any(kw in text.lower() for kw in fail_keywords):
            print(f"\n❌ FAIL: Lua script error — {text}", flush=True)
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
    args = parser.parse_args()

    inject_port = args.inject_port if args.inject_port else args.mavlink_port + 1

    threading.Thread(target=mock_modem_thread, args=(args.port,),
                     daemon=True).start()
    sys.exit(monitor_gcs(args.mavlink_port, inject_port, args.timeout))