#!/usr/bin/env python3
import sys
import time
import socket
import serial
import threading
import argparse

# The exact responses a healthy EC25 modem gives
AT_RESPONSES = {
    b'ATI':        b'\r\nEC25EFAR06A18M4G\r\nOK\r\n',
    b'AT+CPIN?':   b'\r\n+CPIN: READY\r\nOK\r\n',
    b'AT+CEREG?':  b'\r\n+CEREG: 0,1\r\nOK\r\n',
    b'AT+CREG?':   b'\r\n+CREG: 0,1\r\nOK\r\n',
    b'AT+QCSQ':    b'\r\n+QCSQ: "eMTC",-65,-10,18,-8\r\nOK\r\n',
    b'AT+QENG=':   b'\r\n+QENG: "servingcell","LTE","FDD",404,45,1A2B3C,300,1,3,-110,-12,-80,11,-,-,-,-,-,-\r\nOK\r\n',
    b'AT+QISTATE': b'\r\nOK\r\n',
    b'AT+QIOPEN':  b'\r\nOK\r\n+QIOPEN: 0,0\r\n',
    b'AT+QICLOSE': b'\r\nOK\r\n',
}

def mock_modem_thread(port):
    """Listens to the virtual serial port and replies with fake modem data."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"[MOCK] Listening on {port}...")
        buf = b''
        while True:
            data = ser.read(128)
            if data:
                buf += data
                if b'\r' in buf or b'\n' in buf:
                    # Find matching response
                    replied = False
                    for key, resp in AT_RESPONSES.items():
                        if key in buf:
                            time.sleep(0.05) # Simulate slight modem delay
                            ser.write(resp)
                            ser.flush()
                            replied = True
                            break
                    
                    if not replied and b'AT' in buf:
                        # Catch-all for generic AT commands (e.g., ATE0, AT+CFUN=1)
                        time.sleep(0.05)
                        ser.write(b'\r\nOK\r\n')
                        ser.flush()
                    
                    buf = b'' # Clear buffer after line
    except Exception as e:
        print(f"[MOCK] Serial error: {e}")

def monitor_gcs(mavlink_port, timeout=60):
    """Listens to ArduPilot's GCS output to see if the script succeeded."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', mavlink_port))
    sock.settimeout(1.0)
    
    start_time = time.time()
    print(f"[GCS] Monitoring MAVLink on UDP {mavlink_port} for {timeout} seconds...")
    
    while time.time() - start_time < timeout:
        try:
            data, _ = sock.recvfrom(4096)
            text = data.decode('utf-8', errors='ignore')
            
            # Print Lua script progress
            if "LTE_modem:" in text:
                # Clean up the MAVLink packet text formatting slightly for the console
                clean_text = text[text.find("LTE_modem:"):].split('\x00')[0]
                print(f"  -> {clean_text}")
                
                if "connected" in clean_text:
                    print("\n✅ SUCCESS: Basic Flow reached CONNECTED state!")
                    return 0
                if "error" in clean_text or "bad step" in clean_text:
                    print(f"\n❌ FAIL: Lua script crashed! ({clean_text})")
                    return 1
        except socket.timeout:
            continue

    print("\n❌ FAIL: Timed out waiting for CONNECTED state.")
    return 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--mavlink-port", type=int, required=True)
    args = parser.parse_args()

    # Start the fake modem in the background
    threading.Thread(target=mock_modem_thread, args=(args.port,), daemon=True).start()
    
    # Block the main thread monitoring the GCS output
    sys.exit(monitor_gcs(args.mavlink_port))