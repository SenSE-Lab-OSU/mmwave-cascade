#!/usr/bin/env python3
"""
jetson_check.py - pre-flight check before the first capture on the Jetson.

Checks, in order:
  1. serial ports: finds the XDS110 "Application/User UART" and its /dev name
  2. wired interface has 192.168.33.30
  3. DCA1000 answers on 192.168.33.180 (reads FPGA version)
  4. radar answers '?' on the serial port (prints its STATUS line)
  5. UDP receive buffer limit (sysctl)

Run:  python3 jetson_check.py
"""
import socket
import subprocess
import sys
import time

ok_all = True


def result(ok, msg):
    global ok_all
    ok_all &= ok
    print(("  OK   " if ok else "  FAIL ") + msg)


print("== 1. Serial ports")
radar_port = None
try:
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        result(False, "no serial ports found - is the radar USB plugged into the Jetson?")
    for p in ports:
        desc = f"{p.device}  {p.description}  [{p.hwid}]"
        print("       " + desc)
        if "XDS110" in (p.description or "") and "Application" in (p.description or ""):
            radar_port = p.device
    if radar_port is None:
        # fall back: XDS110 exposes two ACM ports; the app UART is usually the first
        acm = [p.device for p in ports if "ACM" in p.device]
        if acm:
            radar_port = sorted(acm)[0]
            print(f"       (no 'Application/User UART' label; assuming {radar_port})")
    result(radar_port is not None, f"radar serial port: {radar_port}")
except ImportError:
    result(False, "pyserial missing: sudo apt install python3-serial")

print("== 2. Wired interface")
try:
    out = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True).stdout
    has = "192.168.33.30" in out
    for line in out.splitlines():
        if "192.168.33.30" in line:
            print("       " + line.strip())
    result(has, "192.168.33.30 configured" if has else
           "192.168.33.30 not configured - run: sudo ./jetson_setup.sh")
except Exception as e:
    result(False, f"ip addr failed: {e}")

print("== 3. DCA1000")
try:
    from dca1000 import DCA1000
    d = DCA1000(verbose=False)
    try:
        d.connect()
        major, minor = d.read_fpga_version()
        result(True, f"DCA1000 answers, FPGA version {major}.{minor}")
    finally:
        d.close()
except Exception as e:
    result(False, f"DCA1000: {e}")

print("== 4. Radar")
if radar_port:
    try:
        import serial
        with serial.Serial(radar_port, 115200, timeout=0.2) as ser:
            ser.reset_input_buffer()
            ser.write(b"?"); ser.flush()
            t0 = time.time(); lines = []
            while time.time() - t0 < 2.0:
                l = ser.readline()
                if l:
                    lines.append(l.decode("ascii", errors="replace").rstrip())
            for l in lines:
                print("       " + l)
            result(bool(lines), "radar answers '?'" if lines else
                   "radar silent - power-cycle it (12 V out 10 s) and re-run")
    except Exception as e:
        result(False, f"serial: {e}  (if 'Permission denied': log out/in after jetson_setup.sh)")
else:
    result(False, "skipped (no serial port)")

print("== 5. UDP receive buffer")
try:
    rmem = int(open("/proc/sys/net/core/rmem_max").read())
    result(rmem >= (1 << 26), f"net.core.rmem_max = {rmem // (1 << 20)} MB")
except Exception as e:
    result(False, f"sysctl: {e}")

print("\nALL OK - run: python3 jetson_loss_test.py --seconds 60" if ok_all
      else "\nFix the FAIL lines above, then re-run this check.")
sys.exit(0 if ok_all else 1)
