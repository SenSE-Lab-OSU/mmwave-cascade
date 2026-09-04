#!/usr/bin/env python3
"""
radar_ctl.py - control the host-triggered cascade radar firmware over UART.

Usage:
  python radar_ctl.py g            # send 'g' (start chirping), then monitor
  python radar_ctl.py s            # send 's' (stop chirping), then monitor
  python radar_ctl.py status       # send '?' -> board prints clock regs + state
  python radar_ctl.py monitor      # just print what the board says

Port defaults: COM5 on Windows, /dev/ttyACM0 elsewhere.
Override with -p, e.g.  python radar_ctl.py g -p COM7
Stop monitoring with Ctrl+C (the radar keeps doing whatever it was doing).

Typical capture sequence (PC or Jetson):
  1. configure DCA1000 + start recording (start_capture.bat / capture script)
  2. python radar_ctl.py g
  3. ... capture ...
  4. python radar_ctl.py s
"""
import argparse
import sys
import time

import serial  # pip install pyserial


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["g", "s", "status", "monitor"],
                    help="g=start chirping, s=stop, status=query, monitor=listen only")
    ap.add_argument("-p", "--port",
                    default=("COM5" if sys.platform.startswith("win") else "/dev/ttyACM0"),
                    help="serial port (default: %(default)s)")
    ap.add_argument("-t", "--monitor-seconds", type=float, default=None,
                    help="seconds to print board output after the command "
                         "(0 = don't monitor, negative = forever)")
    args = ap.parse_args()
    if args.monitor_seconds is None:
        args.monitor_seconds = 3.0 if args.cmd == "status" else 10.0

    with serial.Serial(args.port, 115200, timeout=0.2) as ser:
        tx = {"g": b"g", "s": b"s", "status": b"?"}.get(args.cmd)
        if tx is not None:
            ser.write(tx)
            ser.flush()
            print(f"sent {tx.decode()!r} on {args.port}")

        if args.monitor_seconds == 0:
            return

        t0 = time.time()
        try:
            while args.monitor_seconds < 0 or (time.time() - t0) < args.monitor_seconds:
                line = ser.readline()
                if line:
                    print(line.decode("ascii", errors="replace").rstrip())
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
