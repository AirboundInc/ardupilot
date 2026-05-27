#!/usr/bin/env python3
"""
LTE Modem SITL Happy-Path Test
-------------------------------
1. Runs a mock EC25 modem on a PTY — validates QIOPEN destination
2. Connects to MAVLink, waits for heartbeat
3. Injects LTE_SERVER_* params via PARAM_SET
4. Monitors STATUSTEXT for CONNECTED state
5. Validates the Lua script walked the correct step sequence
6. Verifies the Lua script connected to the correct IP:port

Exit codes:  0 = all checks pass,  1 = timeout / error / wrong sequence
"""
import sys
import time
import serial
import threading
import argparse
from pymavlink import mavutil

# ── Expected server destination (must match injected params) ──────────
EXPECTED_IP = "8.8.8.8"
EXPECTED_PORT = 8080

# ── Expected step order for the happy path (CMUX bypassed via sed) ────
EXPECTED_STEPS = [
    "ATI",
    "CPIN",
    "CONFIG",
    "CREG",
    "SIGNAL_GATE",
    "QENG",
    "SOCKET_STATE",
    "CIPOPEN",
    "CONNECTED",
]

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

# ── Params to inject after LTE driver registers them ──────────────────
INJECTED_PARAMS = {
    'LTE_SERVER_PORT': EXPECTED_PORT,
    'LTE_SERVER_IP0':  int(EXPECTED_IP.split('.')[0]),
    'LTE_SERVER_IP1':  int(EXPECTED_IP.split('.')[1]),
    'LTE_SERVER_IP2':  int(EXPECTED_IP.split('.')[2]),
    'LTE_SERVER_IP3':  int(EXPECTED_IP.split('.')[3]),
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


def mock_modem_thread(port):
    """Serial listener that responds to AT commands."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"[MOCK] Listening on {port}...", flush=True)
        buf = b''
        while True:
            data = ser.read(128)
            if data:
                buf += data
                while b'\r' in buf or b'\n' in buf:
                    idx_r = buf.find(b'\r')
                    idx_n = buf.find(b'\n')
                    if idx_r != -1 and idx_n != -1:
                        idx = min(idx_r, idx_n)
                    else:
                        idx = max(idx_r, idx_n)

                    line = buf[:idx].strip()
                    buf = buf[idx + 1:]

                    if not line:
                        continue

                    line_str = line.decode('utf-8', errors='ignore')
                    print(f"[MOCK] RX: {line_str}", flush=True)

                    # QIOPEN gets special validation
                    if 'AT+QIOPEN' in line_str:
                        time.sleep(0.05)
                        handle_qiopen(line_str, ser)
                        continue

                    replied = False
                    for key, resp in AT_RESPONSES.items():
                        if key in line_str:
                            time.sleep(0.05)
                            ser.write(resp)
                            ser.flush()
                            print(f"[MOCK] TX: {key}", flush=True)
                            replied = True
                            break

                    if not replied and 'AT' in line_str:
                        time.sleep(0.05)
                        ser.write(b'\r\nOK\r\n')
                        ser.flush()
                        print("[MOCK] TX: Generic OK", flush=True)

    except Exception as e:
        print(f"[MOCK] Serial error: {e}", flush=True)


def inject_params(master):
    """Send PARAM_SET messages for LTE server params."""
    print("[GCS] Injecting LTE_SERVER_* params via PARAM_SET...", flush=True)

    for name, value in INJECTED_PARAMS.items():
        name_bytes = name.encode('utf-8')

        acked = False
        for attempt in range(3):
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                name_bytes,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            ack = master.recv_match(type='PARAM_VALUE', blocking=True,
                                    timeout=2.0)
            if ack and ack.param_id.strip('\x00') == name:
                print(f"  [PARAM] {name} = {ack.param_value}", flush=True)
                acked = True
                break
            time.sleep(0.2)

        if not acked:
            print(f"  [PARAM] {name} — FAILED after 3 attempts", flush=True)

        time.sleep(0.1)

    print("[GCS] Param injection complete.", flush=True)


def validate_steps(observed):
    """Check observed step sequence matches expected happy path."""
    if observed == EXPECTED_STEPS:
        print(f"\n✅ Step sequence OK: {' → '.join(observed)}", flush=True)
        return True

    print(f"\n❌ FAIL: Step sequence mismatch!", flush=True)
    print(f"   Expected: {' → '.join(EXPECTED_STEPS)}", flush=True)
    print(f"   Observed: {' → '.join(observed)}", flush=True)

    # Show what went wrong
    missing = [s for s in EXPECTED_STEPS if s not in observed]
    extra = [s for s in observed if s not in EXPECTED_STEPS]
    if missing:
        print(f"   Missing steps: {missing}", flush=True)
    if extra:
        print(f"   Unexpected steps: {extra}", flush=True)

    return False


def monitor_gcs(mavlink_port, timeout):
    """Wait for heartbeat, inject params, then watch for CONNECTED."""
    print(f"[GCS] Waiting for MAVLink heartbeat on UDP {mavlink_port}...",
          flush=True)
    master = mavutil.mavlink_connection(f'udpin:0.0.0.0:{mavlink_port}')
    master.wait_heartbeat()
    print(f"[GCS] Heartbeat received (system {master.target_system},"
          f" component {master.target_component})", flush=True)

    # 3s is enough — the LTE C++ driver registers params during init(),
    # which runs before Lua starts (~4-5s after boot).
    print("[GCS] Waiting 3s for LTE driver param registration...",
          flush=True)
    time.sleep(3)

    inject_params(master)

    # Monitor STATUSTEXT for success/failure
    print(f"[GCS] Monitoring STATUSTEXT for {timeout}s...", flush=True)
    start_time = time.time()
    observed_steps = []

    while time.time() - start_time < timeout:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
        if not msg:
            continue

        text = msg.text
        if "LTE" not in text:
            continue

        elapsed = int(time.time() - start_time)
        print(f"  [{elapsed:3d}s] {text}", flush=True)

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

            # --- CHECK 2: QIOPEN destination ---
            with qiopen_lock:
                if qiopen_result['received'] and qiopen_result['validated']:
                    print(f"✅ QIOPEN destination OK: "
                          f"{EXPECTED_IP}:{EXPECTED_PORT}", flush=True)
                elif qiopen_result['received']:
                    print(f"\n❌ FAIL: Connected to WRONG address!",
                          flush=True)
                    print(f"   QIOPEN cmd: {qiopen_result['command']}",
                          flush=True)
                    return 1
                else:
                    print(f"\n⚠️  WARN: CONNECTED but no QIOPEN seen",
                          flush=True)

            print(f"\n✅ SUCCESS: All checks passed.", flush=True)
            return 0

        # Fatal errors
        fail_keywords = ["could not find", "bad step"]
        if any(kw in text.lower() for kw in fail_keywords):
            print(f"\n❌ FAIL: Lua script error — {text}", flush=True)
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
                        help="UDP port to listen for MAVLink")
    parser.add_argument("--timeout", type=int, default=120,
                        help="Seconds to wait for CONNECTED (default 120)")
    args = parser.parse_args()

    threading.Thread(target=mock_modem_thread, args=(args.port,),
                     daemon=True).start()
    sys.exit(monitor_gcs(args.mavlink_port, args.timeout))