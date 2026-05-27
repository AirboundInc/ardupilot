#!/usr/bin/env python3
"""
LTE Modem SITL Happy-Path Test
-------------------------------
Runs a mock EC25 modem on a PTY and monitors MAVLink STATUSTEXT messages
for the Lua script to reach the CONNECTED state.

Exit codes:  0 = CONNECTED reached,  1 = timeout or script error
"""
import sys
import time
import serial
import threading
import argparse
from pymavlink import mavutil

# ── AT command → response table (happy-path only) ──────────────────────
AT_RESPONSES = {
    'ATI':        b'\r\nEC25EFAR06A18M4G\r\nOK\r\n',
    'AT+CPIN?':   b'\r\n+CPIN: READY\r\nOK\r\n',
    'AT+CEREG?':  b'\r\n+CEREG: 0,1\r\nOK\r\n',
    'AT+CREG?':   b'\r\n+CREG: 0,1\r\nOK\r\n',
    'AT+QCSQ':    b'\r\n+QCSQ: "eMTC",-65,-10,18,-8\r\nOK\r\n',
    'AT+CPSI?':   b'\r\n+CPSI: LTE,Online,404-45,0x0001,1A2B3C,300,EUTRAN-BAND1,1,3,3,-110,-12,-80,11\r\nOK\r\n',
    'AT+QENG=':   b'\r\n+QENG: "servingcell","LTE","FDD",404,45,1A2B3C,300,1,3,-110,-12,-80,11,-,-,-,-,-,-\r\nOK\r\n',
    'AT+QISTATE': b'\r\nOK\r\n',
    'AT+QIOPEN':  b'\r\nOK\r\n+QIOPEN: 0,0\r\n',
    'AT+QICLOSE': b'\r\nOK\r\n',
    'AT+COPS':    b'\r\nOK\r\n',
}


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


def monitor_gcs(mavlink_port, timeout):
    """Watch STATUSTEXT messages for success / failure keywords."""
    print(f"[GCS] Monitoring MAVLink on UDP {mavlink_port} "
          f"for {timeout} seconds...", flush=True)
    master = mavutil.mavlink_connection(f'udpin:0.0.0.0:{mavlink_port}')
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
        if not msg:
            continue

        text = msg.text
        if "LTE" in text:
            elapsed = int(time.time() - start_time)
            print(f"  [{elapsed:3d}s] {text}", flush=True)

            if "connected" in text.lower():
                print("\n✅ SUCCESS: Basic Flow reached CONNECTED state!",
                      flush=True)
                return 0

            fail_keywords = ["could not find", "error", "bad step",
                             "must set"]
            if any(kw in text.lower() for kw in fail_keywords):
                print(f"\n❌ FAIL: Lua script error — {text}", flush=True)
                return 1

    print("\n❌ FAIL: Timed out waiting for CONNECTED state.", flush=True)
    return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Mock EC25 modem + MAVLink monitor for LTE SITL tests")
    parser.add_argument("--port", required=True,
                        help="PTY path for the mock modem (e.g. /tmp/ttyMock)")
    parser.add_argument("--mavlink-port", type=int, required=True,
                        help="UDP port to listen for STATUSTEXT messages")
    parser.add_argument("--timeout", type=int, default=120,
                        help="Seconds to wait for CONNECTED (default: 120)")
    args = parser.parse_args()

    threading.Thread(target=mock_modem_thread, args=(args.port,),
                     daemon=True).start()
    sys.exit(monitor_gcs(args.mavlink_port, args.timeout))