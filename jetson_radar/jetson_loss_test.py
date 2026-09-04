#!/usr/bin/env python3
"""
jetson_loss_test.py - RAM-only radar receiver for the Jetson (step 1: go/no-go).

Does exactly what capture_ddma.py does on the PC, minus the .bin writer and
the display:
  1. configure the DCA1000 (dca1000.py), start recording
  2. send 'g' to the radar over /dev/ttyACM0
  3. receive UDP packets, assemble chirps into frames, count losses
  4. print a stats line every 2 s
  5. on Ctrl+C (or after --seconds): send 's', stop recording, print a summary
     with the same PASS/CHECK verdict as the PC script.

NOTHING is written to disk. A small ring of the most recent frames is kept in
RAM (RING_FRAMES) so the next step (live processing) has data to work on.

Usage:
  python3 jetson_loss_test.py                # run until Ctrl+C
  python3 jetson_loss_test.py --seconds 60   # timed run
  python3 jetson_loss_test.py --no-radar     # DCA1000 only (radar started elsewhere)
"""
import argparse
import socket
import struct
import sys
import threading
import time
from collections import deque

from dca1000 import DCA1000, DCA1000Error

try:
    import serial
except ImportError:
    serial = None

# ============================================================
# CONFIG - must match firmware (identical to capture_ddma.py)
# ============================================================
SAMPLES_PER_CHIRP = 256
NUM_RX            = 8
CHIRPS_PER_FRAME  = 64
EXPECTED_DT_MS    = 1000.0 / 120.0

HEADER_MAGIC   = 0xA1B2C3D4
HEADER_BYTES   = 16
MAGIC_LE       = struct.pack("<I", HEADER_MAGIC)
BYTES_PER_SAMPLE = 4
DATA_PER_CHIRP = SAMPLES_PER_CHIRP * NUM_RX * BYTES_PER_SAMPLE   # 8192
BLOCK_BYTES    = DATA_PER_CHIRP + HEADER_BYTES                   # 8208
FRAME_BYTES    = CHIRPS_PER_FRAME * BLOCK_BYTES                  # 525,312

FPGA_IP, HOST_IP       = "192.168.33.180", "192.168.33.30"
CONFIG_PORT, DATA_PORT = 4096, 4098
DCA_HEADER_BYTES       = 10
SOCKET_RECV_BUF        = 1 << 26          # 64 MB (needs net.core.rmem_max >= this)
DCA_PACKET_DELAY_US    = 5

RADAR_PORT = "/dev/ttyACM0" if not sys.platform.startswith("win") else "COM5"
RADAR_BAUD = 115200

RING_FRAMES  = 8          # frames kept in RAM (8 x 525 KB = 4.2 MB)
STATS_PERIOD = 2.0        # seconds between stats lines


class RadarLink:
    def __init__(self, port=RADAR_PORT, baud=RADAR_BAUD):
        if serial is None:
            raise RuntimeError("pyserial missing: sudo apt install python3-serial")
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.ser.reset_input_buffer()

    def _send_and_echo(self, byte, seconds):
        self.ser.write(byte); self.ser.flush()
        t0 = time.time()
        while time.time() - t0 < seconds:
            line = self.ser.readline()
            if line:
                print("  radar:", line.decode("ascii", errors="replace").rstrip())

    def go(self):    self._send_and_echo(b"g", 2.0)
    def stop(self):  self._send_and_echo(b"s", 3.0)
    def close(self): self.ser.close()


class Receiver(threading.Thread):
    """recv -> deque, nothing else (same as the PC script)."""
    def __init__(self, sock):
        super().__init__(daemon=True)
        self.sock = sock
        self.q = deque()
        self.stop_flag = threading.Event()
        self.first_pkt_time = None

    def run(self):
        recv = self.sock.recv
        q_append = self.q.append
        while not self.stop_flag.is_set():
            try:
                pkt = recv(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.first_pkt_time is None:
                self.first_pkt_time = time.perf_counter()
            q_append(pkt)


def run(seconds, use_radar):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RECV_BUF)
    got_buf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    sock.bind((HOST_IP, DATA_PORT)); sock.settimeout(0.5)
    print(f"Listening on {HOST_IP}:{DATA_PORT}  (socket rcvbuf {got_buf // 1024} KB"
          f"{' - LOW, run jetson_setup.sh' if got_buf < SOCKET_RECV_BUF else ''})")
    print("RAM-only: nothing is written to disk.")

    rx = Receiver(sock)
    rx.start()

    dca = DCA1000(FPGA_IP, HOST_IP, CONFIG_PORT)
    dca.configure(packet_delay_us=DCA_PACKET_DELAY_US)
    dca.start_record()

    radar = None
    if use_radar:
        radar = RadarLink()
        print(f"Radar: sending 'g' on {RADAR_PORT}")
        radar.go()
    print(f"Receiving for {'ever' if seconds <= 0 else str(seconds) + ' s'} "
          f"(Ctrl+C to stop)...\n")

    # ---- state (same names/semantics as capture_ddma.py) ----
    ring = deque(maxlen=RING_FRAMES)     # (frame_id, got, bytes) newest last
    buf = bytearray()
    scan = 0
    base_offset = 0
    cur_frame = None
    got = 0
    frame_offset = None
    prev_gcount = None
    chirps_dropped = 0
    dca_pkts = dca_lost = 0
    expected_seq = None
    prev_bt = None
    max_qlen = 0
    frames = 0
    frames_missing = 0
    dt_sum = 0.0
    dt_min, dt_max = 1e9, 0.0
    prev_frame_offset = None
    prev_frame_id = None
    bad_off_steps = bad_fid_steps = 0
    bytes_missing = 0
    last_stats = time.perf_counter()
    stats_frames = 0
    stats_lost = 0

    def finalize(frame_id, got_count, frame_bytes):
        nonlocal prev_frame_offset, prev_frame_id, bad_off_steps, bad_fid_steps
        nonlocal bytes_missing, frames, frames_missing, dt_sum, dt_min, dt_max
        nonlocal stats_frames
        dt = (time.perf_counter() - prev_bt) * 1e3 if prev_bt is not None else 0.0
        off_step = (frame_offset - prev_frame_offset
                    if prev_frame_offset is not None else None)
        if off_step is not None and off_step != FRAME_BYTES:
            bad_off_steps += 1
            if off_step < FRAME_BYTES:
                bytes_missing += FRAME_BYTES - off_step
        fid_step = (frame_id - prev_frame_id if prev_frame_id is not None else None)
        if fid_step is not None and fid_step != 1:
            bad_fid_steps += 1
        prev_frame_offset = frame_offset
        prev_frame_id = frame_id
        frames += 1
        stats_frames += 1
        if got_count < CHIRPS_PER_FRAME:
            frames_missing += 1
        if frames > 1:
            dt_sum += dt; dt_min = min(dt_min, dt); dt_max = max(dt_max, dt)
        ring.append((frame_id, got_count, frame_bytes))

    q = rx.q
    t_start = time.perf_counter()
    try:
        while True:
            if seconds > 0 and time.perf_counter() - t_start >= seconds:
                break
            now = time.perf_counter()
            if now - last_stats >= STATS_PERIOD:
                el = now - last_stats
                print(f"frames {frames:6d}  {stats_frames / el:6.1f} fps  "
                      f"dca_lost +{stats_lost:<5d} (total {dca_lost})  "
                      f"chirps_dropped {chirps_dropped}  queue {len(q)} (peak {max_qlen})",
                      flush=True)
                last_stats = now; stats_frames = 0; stats_lost = 0
            if not q:
                time.sleep(0.001)
                continue

            n = len(q)
            if n > max_qlen:
                max_qlen = n
            for _ in range(n):
                pkt = q.popleft()
                if len(pkt) <= DCA_HEADER_BYTES:
                    continue
                seq = struct.unpack("<I", pkt[0:4])[0]
                dca_pkts += 1
                if expected_seq is not None and seq > expected_seq:
                    dca_lost += seq - expected_seq
                    stats_lost += seq - expected_seq
                expected_seq = seq + 1
                buf.extend(pkt[DCA_HEADER_BYTES:])

            idx = buf.find(MAGIC_LE, scan)
            while idx != -1:
                if idx + HEADER_BYTES > len(buf):
                    break
                if idx >= DATA_PER_CHIRP:
                    _, gcount, fid, cid = struct.unpack("<4I", buf[idx:idx + HEADER_BYTES])
                    if prev_gcount is not None and gcount > prev_gcount + 1:
                        chirps_dropped += gcount - prev_gcount - 1
                    prev_gcount = gcount
                    if fid != cur_frame or got >= CHIRPS_PER_FRAME:
                        t = time.perf_counter()
                        if cur_frame is not None:
                            # frame bytes = everything from its first header to here
                            start = frame_offset - base_offset
                            end = idx - DATA_PER_CHIRP
                            fb = bytes(buf[start:end]) if 0 <= start < end <= len(buf) else b""
                            finalize(cur_frame, got, fb)
                        prev_bt = t
                        cur_frame = fid
                        got = 0
                        frame_offset = base_offset + idx - DATA_PER_CHIRP
                    got += 1
                scan = idx + HEADER_BYTES
                idx = buf.find(MAGIC_LE, scan)

            # keep only the current (open) frame in the working buffer
            keep_from = (frame_offset - base_offset) if frame_offset is not None else scan
            keep_from = max(0, min(keep_from, scan))
            if keep_from > 0:
                base_offset += keep_from
                del buf[:keep_from]
                scan -= keep_from

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        rx.stop_flag.set()
        if radar is not None:
            print("Radar: sending 's'")
            try:
                radar.stop()
            finally:
                radar.close()
        try:
            dca.stop_record()
        except DCA1000Error as e:
            print(e)
        dca.close()
        rx.join(timeout=2.0)
        sock.close()

    # ---- summary (same verdict logic as the PC script) ----
    print("\n--- Summary ---")
    print(f"Frames received:      {frames}")
    if frames > 1:
        mean_dt = dt_sum / (frames - 1)
        print(f"Mean dt:              {mean_dt:.2f} ms (target {EXPECTED_DT_MS:.0f} ms, "
              f"{1000.0 / mean_dt:.1f} fps)")
        print(f"Min / Max dt:         {dt_min:.2f} / {dt_max:.2f} ms")
    print(f"Chirps dropped:       {chirps_dropped}")
    print(f"DCA packets:          {dca_pkts}   lost: {dca_lost}")
    print(f"Frames w/ missing:    {frames_missing}")
    print(f"Peak queue depth:     {max_qlen} packets")
    print("\n--- Stream contiguity ---")
    print(f"Bad byte steps:       {bad_off_steps}")
    print(f"Bytes missing:        {bytes_missing}")
    print(f"Bad frame-id steps:   {bad_fid_steps}")
    print(f"Frames in RAM ring:   {len(ring)} (latest id "
          f"{ring[-1][0] if ring else '-'}, {sum(len(r[2]) for r in ring) // 1024} KB)")
    ok = (frames > 10 and chirps_dropped == 0 and dca_lost == 0
          and frames_missing == 0 and bad_off_steps == 0 and bad_fid_steps == 0)
    if ok:
        print(f"\nVERDICT: PASS - holds {EXPECTED_DT_MS:.0f} ms with zero loss, "
              "byte stream contiguous, RAM-only.")
    else:
        print("\nVERDICT: CHECK - see stats above.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop automatically after this many seconds (0 = Ctrl+C)")
    ap.add_argument("--no-radar", action="store_true",
                    help="don't touch the serial port (radar started some other way)")
    a = ap.parse_args()
    run(a.seconds, not a.no_radar)
