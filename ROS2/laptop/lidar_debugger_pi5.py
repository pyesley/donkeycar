#!/usr/bin/env python3
"""
rplidar_debugger.py — Standalone RPLiDAR serial-protocol debugger
Tested with Python 3.10+ and pyserial on Raspberry Pi.

Usage examples:
  python3 rplidar_debugger.py --port /dev/ttyUSB0 --bauds 256000,115200 --enable-motor --scan 720
  python3 rplidar_debugger.py --auto --scan 360
  python3 rplidar_debugger.py --port /dev/ttyUSB0 --just-info

What it does:
  • Enumerates candidate serial ports (if --auto)
  • Tries one or more baud rates (256000, 115200 by default)
  • Toggles DTR/RTS to enable/disable motor (USB adapter boards use DTR)
  • Sends GET_INFO, GET_HEALTH, START_SCAN; reads response headers & prints raw hex
  • Optionally decodes legacy 5-byte nodes to angles (deg) and distances (m)
  • Writes an inspection log to rplidar_debug.log in the current directory

Notes:
  • If you see only timeouts at all bauds, the common culprits are:
      - Wrong baud (try 256000 for A2/A3; 115200 for some A1/S1 variants)
      - Bad cable or insufficient power (use a powered USB hub)
      - DTR motor gating reversed (try --enable-motor / --disable-motor)
      - Port permissions (use user in 'dialout' group or run with sudo)

(c) helper script for debugging, no warranties. — ChatGPT
"""
import argparse
import sys
import time
import glob
import os
from typing import Optional, Tuple, List

try:
    import serial
except Exception as e:
    print("This script requires 'pyserial'. Install with: pip3 install pyserial", file=sys.stderr)
    raise

LOGFILE = "rplidar_debug.log"

# ---- RPLIDAR protocol constants (subset) ----
SYNC_BYTE = 0xA5
RESPONSE_SYNC_1 = 0xA5
RESPONSE_SYNC_2 = 0x5A

CMD_STOP         = 0x25
CMD_RESET        = 0x40
CMD_SCAN         = 0x20
CMD_GET_INFO     = 0x50
CMD_GET_HEALTH   = 0x52

# Legacy scan node packet is 5 bytes (A1/A2 class), express modes use different formats.
LEGACY_NODE_SIZE = 5

# ---- Utilities ----
def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def log(msg: str):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    try:
        with open(LOGFILE, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass

def list_candidate_ports() -> List[str]:
    # Typical USB serial device names
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*"]
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    # Deduplicate & sort consistently
    ports = sorted(set(ports))
    return ports

def open_port(port: str, baud: int, timeout: float=0.2) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = timeout
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.bytesize = serial.EIGHTBITS
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.open()
    return ser

def set_motor_lines(ser: serial.Serial, enable: Optional[bool]=None, rts: Optional[bool]=None):
    """
    Many RPLIDAR USB adapters gate motor power using DTR (sometimes RTS).
    On your setup the C++ node logged:
       'DTR CLEARED - motor enabled' and 'DTR SET - motor disabled'.
    In pyserial, setting ser.dtr=False 'clears' DTR; True 'sets' DTR.
    """
    if enable is not None:
        # Clear DTR -> enable; Set DTR -> disable (per your board)
        ser.dtr = (not enable)
        log(f"DTR set to {'LOW (cleared)' if ser.dtr==False else 'HIGH (set)'}  -> motor {'ENABLED' if enable else 'DISABLED'}")
    if rts is not None:
        ser.rts = bool(rts)
        log(f"RTS set to {'HIGH' if ser.rts else 'LOW'}")

def send_cmd(ser: serial.Serial, cmd: int, payload: bytes=b""):
    """
    RPLIDAR commands: 0xA5 <cmd> [optional payload length + payload + checksum]
    For GET_INFO/GET_HEALTH/SCAN/STOP/RESET, payload is empty.
    """
    frame = bytes([SYNC_BYTE, cmd])
    if payload:
        frame += bytes([len(payload)]) + payload + bytes([checksum(payload)])
    ser.write(frame)
    ser.flush()
    log(f"TX: {hexdump(frame)}")

def checksum(payload: bytes) -> int:
    s = 0
    for b in payload:
        s ^= b
    return s & 0xFF

def read_descriptor(ser: serial.Serial, timeout_s: float=2.0) -> Optional[Tuple[int,int]]:
    """
    Read the 7-byte response descriptor header:
      A5 5A <len0> <len1> <len2> <len3> <type>
    Returns (data_len, data_type) or None on timeout.
    """
    deadline = time.time() + timeout_s
    # Hunt for sync bytes
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != RESPONSE_SYNC_1:
            continue
        b2 = ser.read(1)
        if not b2:
            continue
        if b2[0] != RESPONSE_SYNC_2:
            continue
        rest = ser.read(5)
        if len(rest) != 5:
            continue
        desc = bytes([RESPONSE_SYNC_1, RESPONSE_SYNC_2]) + rest
        log(f"RX DESC: {hexdump(desc)}")
        l0, l1, l2, l3, dtype = rest[0], rest[1], rest[2], rest[3], rest[4]
        data_len = l0 | (l1<<8) | (l2<<16) | (l3<<24)
        return data_len, dtype
    return None

def read_exact(ser: serial.Serial, n: int, timeout_s: float=2.0) -> Optional[bytes]:
    buf = b""
    deadline = time.time() + timeout_s
    while len(buf) < n and time.time() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return buf if len(buf) == n else None

def get_device_info(ser: serial.Serial) -> Optional[bytes]:
    send_cmd(ser, CMD_GET_INFO)
    desc = read_descriptor(ser, timeout_s=2.0)
    if not desc:
        log("ERROR: No descriptor for GET_INFO (timeout)")
        return None
    length, dtype = desc
    # Device info is typically 20 bytes
    data = read_exact(ser, length, timeout_s=2.0)
    log("RX INFO: " + (hexdump(data) if data else "None"))
    if data and len(data) >= 20:
        serialno = data[0:16]
        model = data[16]
        fw_minor = data[17]
        fw_major = data[18]
        hwrev = data[19]
        log(f"Parsed INFO: model={model} fw={fw_major}.{fw_minor} hwrev={hwrev} serial={serialno.hex()}")
    return data

def get_health(ser: serial.Serial) -> Optional[bytes]:
    send_cmd(ser, CMD_GET_HEALTH)
    desc = read_descriptor(ser, timeout_s=2.0)
    if not desc:
        log("ERROR: No descriptor for GET_HEALTH (timeout)")
        return None
    length, dtype = desc
    # Health payload is typically 3 bytes: status(2B?) + error code? (varies by model)
    data = read_exact(ser, length, timeout_s=2.0)
    log("RX HEALTH: " + (hexdump(data) if data else "None"))
    return data

def stop_scan(ser: serial.Serial):
    send_cmd(ser, CMD_STOP)
    time.sleep(0.05)  # short wait
    ser.reset_input_buffer()

def reset_lidar(ser: serial.Serial):
    send_cmd(ser, CMD_RESET)
    # Reset will cause the device to reboot; give it time and flush garbage
    time.sleep(2.0)
    ser.reset_input_buffer()

def start_scan(ser: serial.Serial) -> bool:
    send_cmd(ser, CMD_SCAN)
    desc = read_descriptor(ser, timeout_s=2.0)
    if not desc:
        log("ERROR: No descriptor for SCAN (timeout)")
        return False
    length, dtype = desc
    # For legacy scan, descriptor type is often 0x81 with 'length' indicating stream
    log(f"SCAN descriptor: data_len={length} type=0x{dtype:02X} (expect streaming)")
    return True

def decode_legacy_node(node: bytes) -> Optional[Tuple[bool, int, float, float]]:
    """
    Decode a 5-byte legacy measurement node.
    Returns (new_scan, quality, angle_deg, distance_m). Best-effort parsing.
    """
    if len(node) != 5:
        return None
    b0, b1, b2, b3, b4 = node
    # Start flag & check bit (per A1 datasheet): bit0 should be 1 for 'start of new scan'
    start_flag = (b0 & 0x01) == 0x01
    check_bit  = (b0 & 0x02) == 0x02
    # Quality: upper 6 bits of b0
    quality = b0 >> 2
    # Angle (q6) over two bytes, with bit0 of b1 typically the inverse of start flag (consistency check)
    angle_q6 = ((b1 >> 1) | (b2 << 7)) & 0x7FFF
    angle_deg = angle_q6 / 64.0
    # Distance (q2) in mm over two bytes
    dist_q2 = (b3 | (b4 << 8))
    distance_m = (dist_q2 / 4.0) / 1000.0
    return (start_flag and not check_bit, quality, angle_deg, distance_m)

def read_some_scan_nodes(ser: serial.Serial, count: int=360, per_read_timeout: float=2.0) -> int:
    """
    Reads up to 'count' legacy nodes and prints a small summary.
    Returns number of valid nodes decoded.
    """
    valid = 0
    min_d, max_d = 1e9, -1e9
    s0 = time.time()
    while valid < count and (time.time() - s0) < per_read_timeout*max(2, count//200):
        raw = read_exact(ser, LEGACY_NODE_SIZE, timeout_s=per_read_timeout)
        if not raw:
            log("WARN: timeout reading legacy node")
            break
        # Show occasional hex to prove life
        if valid % 60 == 0:
            log("RX NODE: " + hexdump(raw))
        dec = decode_legacy_node(raw)
        if dec is None:
            continue
        new_scan, quality, ang, dist = dec
        if dist > 0:
            min_d = min(min_d, dist)
            max_d = max(max_d, dist)
        valid += 1
    if valid > 0:
        log(f"Scan sample summary: {valid} nodes, distance range ~ {min_d:.2f} m to {max_d:.2f} m")
    else:
        log("No valid nodes decoded")
    return valid

def try_session(port: str, baud: int, enable_motor: Optional[bool], just_info: bool, scan_nodes: int) -> bool:
    log("="*72)
    log(f"Trying port {port} @ {baud} baud")
    try:
        ser = open_port(port, baud)
    except Exception as e:
        log(f"ERROR: could not open {port} @ {baud}: {e}")
        return False
    log(f"Opened {port} @ {baud}")
    # Configure motor gating lines
    try:
        set_motor_lines(ser, enable=enable_motor)
        # small pause to let motor stabilize
        time.sleep(0.2)
    except Exception as e:
        log(f"WARN: setting DTR/RTS failed: {e}")
    # Clean start
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    stop_scan(ser)
    # Some devices need a reset between baud switches
    reset_lidar(ser)

    ok = True
    info = get_device_info(ser)
    health = get_health(ser)
    if info is None:
        log("DIAG: GET_INFO failed. If all bauds fail, suspect wrong baud/cable/power.")
        ok = False
    if health is None:
        log("DIAG: GET_HEALTH failed. Device may be in error or not speaking at this baud.")
        ok = False
    if just_info:
        ser.close()
        return ok

    if not start_scan(ser):
        log("DIAG: SCAN start failed at this baud.")
        ok = False
    else:
        if scan_nodes > 0:
            n = read_some_scan_nodes(ser, count=scan_nodes)
            if n == 0:
                ok = False
        stop_scan(ser)

    # Stop motor unless user asked to keep it running
    try:
        set_motor_lines(ser, enable=False)
    except Exception:
        pass

    ser.close()
    return ok

def main():
    ap = argparse.ArgumentParser(description="RPLiDAR debugger (serial protocol)")
    ap.add_argument("--port", default=None, help="Serial port path (e.g., /dev/ttyUSB0). If omitted, tries --auto")
    ap.add_argument("--auto", action="store_true", help="Probe all /dev/ttyUSB* and /dev/ttyACM* ports")
    ap.add_argument("--bauds", default="256000,115200", help="Comma-separated baud list to try")
    ap.add_argument("--enable-motor", action="store_true", help="Enable motor (clear DTR) before commands")
    ap.add_argument("--disable-motor", action="store_true", help="Force motor disabled (set DTR)")
    ap.add_argument("--just-info", action="store_true", help="Only query GET_INFO and GET_HEALTH; no scan")
    ap.add_argument("--scan", type=int, default=360, help="Number of legacy nodes to read if scanning (>0)")
    args = ap.parse_args()

    with open(LOGFILE, "w") as f:
        f.write("RPLIDAR DEBUG LOG\n")

    ports = []
    if args.port:
        ports = [args.port]
    elif args.auto:
        ports = list_candidate_ports()
        log(f"Auto-found ports: {ports}")
    else:
        ap.print_help()
        print("\nTip: use --auto to scan ports, or --port /dev/ttyUSB0")
        return 2

    bauds = []
    try:
        bauds = [int(b.strip()) for b in args.bauds.split(",") if b.strip()]
    except Exception:
        print("Bad --bauds list; expected comma-separated ints, e.g. 256000,115200")
        return 2

    enable_motor = None
    if args.enable_motor and args.disable_motor:
        print("Choose only one of --enable-motor or --disable-motor", file=sys.stderr)
        return 2
    if args.enable_motor:
        enable_motor = True
    if args.disable_motor:
        enable_motor = False

    any_ok = False
    for port in ports:
        for baud in bauds:
            ok = try_session(
                port=port,
                baud=baud,
                enable_motor=enable_motor,
                just_info=args.just_info,
                scan_nodes=max(0, args.scan),
            )
            if ok:
                any_ok = True

    # Final diagnostics/suggestions
    log("="*72)
    if any_ok:
        log("Summary: At least one (port, baud) combo produced valid responses.")
    else:
        log("Summary: No (port, baud) combos produced valid responses.")
        log("Suggestions:")
        log("  • Try switching bauds (256000 vs 115200).")
        log("  • Check motor gating polarity: --enable-motor vs --disable-motor.")
        log("  • Use a powered USB hub; avoid long cables.")
        log("  • Verify permissions: user in 'dialout' group or run with sudo.")
        log("  • If the adapter has a separate 5V motor pin, ensure it’s powered.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
