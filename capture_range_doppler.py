"""
capture_and_display.py  (v6 - range profile + small Doppler, light read)

Capture half: unchanged, runs in its own process exactly like the trusted
capture_and_save.py. Only adds a non-blocking offset handoff to the display.

Display half (separate process): the v4 range-profile line is back, plus a
small range-Doppler map under it. To keep capture loss-free it reads only the
first DISPLAY_LOOPS loops of each frame (a small contiguous slice, ~0.8 MB)
instead of the whole 3 MB frame, and builds both views from that slice.

Axis calibration is filled in from the chirp profile:
  77 GHz start, 75.03 MHz/us slope, 10000 ksps ADC, chirp slot 7+28 = 35 us.
  -> ~7.8 cm range bins (~20 m max), velocity span ~+/-4.6 m/s.
"""

import csv
import datetime
import os
import queue
import socket
import struct
import threading
import time
import multiprocessing as mp
from collections import deque

import numpy as np

# ============================================================
# CONFIG - must match firmware
# ============================================================
SAMPLES_PER_CHIRP = 256
NUM_RX            = 8
NUM_TX            = 6
NUM_LOOPS         = 64
CHIRPS_PER_FRAME  = NUM_LOOPS * NUM_TX          # 384

EXPECTED_DT_MS = 50.0      # fast rate (20 fps)
RUN_SECONDS    = 60
OUT_DIR        = "."

# ---- chirp block format (must match firmware) ----
HEADER_MAGIC = 0xA1B2C3D4
HEADER_BYTES = 16
MAGIC_LE     = struct.pack("<I", HEADER_MAGIC)
BYTES_PER_SAMPLE = 4
DATA_PER_CHIRP   = SAMPLES_PER_CHIRP * NUM_RX * BYTES_PER_SAMPLE   # 8192
BLOCK_BYTES      = DATA_PER_CHIRP + HEADER_BYTES                   # 8208

# ---- network ----
FPGA_IP, HOST_IP = "192.168.33.180", "192.168.33.30"
CONFIG_PORT, DATA_PORT = 4096, 4098
DCA_HEADER_BYTES = 10
SOCKET_RECV_BUF = 2 ** 26
CMD_START_RECORD, CMD_STOP_RECORD = 0x05, 0x06

FILE_BUF_BYTES = 1 << 22   # 4 MB file buffer

# ---- live display (separate process) ----
DISPLAY_PERIOD = 1.0      # seconds between redraws (slow on purpose)
DISPLAY_LOOPS  = 16        # loops read per redraw -> 16*6 = 96 chirps (~0.8 MB)
DYN_RANGE_DB   = 40        # color span below the peak, dB
LAYOUT_B       = False     # int16 interleave guess: flip to True if range looks wrong

# ---- axis calibration (from rlProfileCfg) ----
C_LIGHT         = 299792458.0
ADC_RATE_KSPS   = 10000.0   # digOutSampleRate
SLOPE_MHZ_US    = 75.03     # freqSlopeConst = 1554
START_FREQ_GHZ  = 77.0      # startFreqConst
CHIRP_PERIOD_US = 35.0      # idleTime (7) + rampEndTime (28)


def dca_command(code, data=b""):
    return (struct.pack("<H", 0xA55A) + struct.pack("<H", code)
            + struct.pack("<H", len(data)) + data + struct.pack("<H", 0xEEAA))


def send_config_command(code):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST_IP, CONFIG_PORT)); s.settimeout(2.0)
    s.sendto(dca_command(code), (FPGA_IP, CONFIG_PORT)); s.close()


# ============================================================
# Receiver thread: recv -> queue, nothing else
# ============================================================
class Receiver(threading.Thread):
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


# ============================================================
# Capture (main thread of capture process) - trusted flow + offset handoff
# ============================================================
def capture(bin_path, pub_q):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RECV_BUF)
    sock.bind((HOST_IP, DATA_PORT)); sock.settimeout(0.5)
    print(f"Listening on {HOST_IP}:{DATA_PORT}")
    print(f"Saving raw stream to {bin_path}")

    rx = Receiver(sock)
    rx.start()

    send_config_command(CMD_START_RECORD)
    print("Streaming started. Resume the R5F core in CCS now.")
    print(f"Capturing for {RUN_SECONDS} s (Ctrl+C to stop early)...\n")

    rows = []
    buf = bytearray()
    scan = 0
    base_offset = 0

    cur_frame = None
    got = 0
    frame_offset = None
    frame_wall = None
    prev_gcount = None
    chirps_dropped = 0
    dca_pkts = dca_lost = 0
    expected_seq = None
    prev_bt = None
    max_qlen = 0

    f = open(bin_path, "wb", buffering=FILE_BUF_BYTES)

    def finalize(frame_id, got_count):
        dt = (time.perf_counter() - prev_bt) * 1e3 if prev_bt is not None else 0.0
        rows.append({
            "frame_id": frame_id,
            "wall_time": frame_wall,
            "dt_ms": dt,
            "got": got_count,
            "missing": CHIRPS_PER_FRAME - got_count,
            "dropped_total": chirps_dropped,
            "dca_pkts": dca_pkts,
            "dca_lost": dca_lost,
            "bin_offset": frame_offset,
        })
        try:
            pub_q.put_nowait((frame_id, frame_offset, got_count))
        except Exception:
            pass

    q = rx.q
    try:
        while True:
            if (rx.first_pkt_time is not None
                    and time.perf_counter() - rx.first_pkt_time >= RUN_SECONDS):
                break
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
                expected_seq = seq + 1

                payload = pkt[DCA_HEADER_BYTES:]
                f.write(payload)
                buf.extend(payload)

            idx = buf.find(MAGIC_LE, scan)
            while idx != -1:
                if idx + HEADER_BYTES > len(buf):
                    break
                if idx >= DATA_PER_CHIRP:
                    _, gcount, fid, cid = struct.unpack(
                        "<4I", buf[idx:idx + HEADER_BYTES])

                    if prev_gcount is not None and gcount > prev_gcount + 1:
                        chirps_dropped += gcount - prev_gcount - 1
                    prev_gcount = gcount

                    if fid != cur_frame:
                        t = time.perf_counter()
                        if cur_frame is not None:
                            finalize(cur_frame, got)
                        prev_bt = t
                        cur_frame = fid
                        got = 0
                        frame_wall = time.time()
                        frame_offset = base_offset + idx - DATA_PER_CHIRP
                    got += 1

                scan = idx + HEADER_BYTES
                idx = buf.find(MAGIC_LE, scan)

            if scan > 0:
                base_offset += scan
                del buf[:scan]
                scan = 0
    except KeyboardInterrupt:
        print("Stopped early by user.")
    finally:
        rx.stop_flag.set()
        send_config_command(CMD_STOP_RECORD)
        rx.join(timeout=2.0)
        sock.close()
        f.close()

    print(f"Capture done: {len(rows)} complete frames recorded.")
    print(f"Peak queue depth: {max_qlen} packets "
          "(how far disk lagged the network - small is good)")
    return rows


# ============================================================
# Display process - own GIL, own main thread, light file reader
# ============================================================
def _range_axis():
    fs = ADC_RATE_KSPS * 1e3
    S  = SLOPE_MHZ_US * 1e12
    dR = C_LIGHT * fs / (2.0 * S * SAMPLES_PER_CHIRP)
    return np.arange(SAMPLES_PER_CHIRP) * dR


def _velocity_axis(n_loops):
    lam = C_LIGHT / (START_FREQ_GHZ * 1e9)
    pri = NUM_TX * CHIRP_PERIOD_US * 1e-6        # per-TX interval (interleaved)
    vmax = lam / (4.0 * pri)
    return np.linspace(-vmax, vmax, n_loops, endpoint=False)


def _process(raw, n_loops, layout_b, rwin, dwin):
    """Slice of the frame -> (range_profile, range_doppler_map)."""
    nch = n_loops * NUM_TX
    m = np.frombuffer(raw, dtype=np.uint8).reshape(nch, BLOCK_BYTES)
    data = np.ascontiguousarray(m[:, :DATA_PER_CHIRP])      # strip 16-byte headers
    iq = data.view(np.int16).astype(np.float32)             # (nch, 4096)
    c = iq[:, 0::2] + 1j * iq[:, 1::2]                       # (nch, 2048)
    if not layout_b:
        c = c.reshape(nch, NUM_RX, SAMPLES_PER_CHIRP)                    # LAYOUT A
    else:
        c = c.reshape(nch, SAMPLES_PER_CHIRP, NUM_RX).transpose(0, 2, 1) # LAYOUT B

    c = c.reshape(n_loops, NUM_TX, NUM_RX, SAMPLES_PER_CHIRP)  # chirp = loop*NUM_TX + tx
    rng = np.fft.fft(c * rwin[None, None, None, :], axis=3)    # range FFT

    prof = np.abs(rng).mean(axis=(0, 1, 2))                   # range profile (samples,)

    dop = np.fft.fft(rng * dwin[:, None, None, None], axis=0)  # Doppler over loops
    dop = np.fft.fftshift(dop, axes=0)
    rd = np.abs(dop).sum(axis=(1, 2)).T                       # (range, loops)
    return prof, rd


def display_proc(bin_path, pub_q, stop_evt):
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"display: matplotlib unavailable ({e}); capturing without a live view.")
        return

    while not os.path.exists(bin_path):
        if stop_evt.is_set():
            return
        time.sleep(0.1)

    r = _range_axis()
    v = _velocity_axis(DISPLAY_LOOPS)
    rwin = np.hanning(SAMPLES_PER_CHIRP).astype(np.float32)
    dwin = np.hanning(DISPLAY_LOOPS).astype(np.float32)
    need = DISPLAY_LOOPS * NUM_TX * BLOCK_BYTES

    fr = open(bin_path, "rb")
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7))

    (ln,) = ax1.plot(r, np.zeros_like(r))
    ax1.set_xlabel("range (m)")
    ax1.set_ylabel("magnitude (a.u.)")
    ax1.set_title("range profile")

    img = ax2.imshow(np.zeros((SAMPLES_PER_CHIRP, DISPLAY_LOOPS), dtype=np.float32),
                     aspect="auto", origin="lower", cmap="viridis",
                     extent=[v[0], v[-1], r[0], r[-1]])
    ax2.set_xlabel("velocity (m/s)")
    ax2.set_ylabel("range (m)")
    ax2.set_title("range-Doppler")
    fig.colorbar(img, ax=ax2, label="dB")
    fig.tight_layout()

    try:
        while not stop_evt.is_set():
            if not plt.fignum_exists(fig.number):
                break

            item = None
            try:
                while True:
                    item = pub_q.get_nowait()      # newest frame only
            except queue.Empty:
                pass
            if item is None:
                plt.pause(DISPLAY_PERIOD)
                continue

            fid, off, got = item
            if got < DISPLAY_LOOPS * NUM_TX:        # need at least our slice
                plt.pause(DISPLAY_PERIOD)
                continue

            fr.seek(off)
            raw = fr.read(need)
            if len(raw) < need:                     # not flushed yet
                plt.pause(DISPLAY_PERIOD)
                continue

            prof, rd = _process(raw, DISPLAY_LOOPS, LAYOUT_B, rwin, dwin)
            ln.set_ydata(prof)
            ax1.relim()
            ax1.autoscale_view(scalex=False)

            rd_db = 20.0 * np.log10(rd + 1e-6)
            peak = float(rd_db.max())
            img.set_data(rd_db)
            img.set_clim(peak - DYN_RANGE_DB, peak)
            plt.pause(DISPLAY_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        fr.close()
        try:
            plt.ioff()
            plt.close(fig)
        except Exception:
            pass


# ============================================================
# CSV timing log
# ============================================================
def write_csv(rows, path):
    with open(path, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["frame_id", "wall_clock", "dt_ms", "chirps_got", "missing",
                    "chirps_dropped_total", "dca_pkts", "dca_lost", "bin_offset"])
        for r in rows:
            ts = datetime.datetime.fromtimestamp(r["wall_time"]).strftime(
                "%Y-%m-%d %H:%M:%S.%f")[:-3]
            w.writerow([r["frame_id"], ts, f"{r['dt_ms']:.2f}", r["got"],
                        r["missing"], r["dropped_total"], r["dca_pkts"],
                        r["dca_lost"], r["bin_offset"]])
    print(f"Wrote {path}")


# ============================================================
# Verify file on disk matches live stats
# ============================================================
def verify_bin(path, live_rows):
    size = os.path.getsize(path)
    frames = {}
    with open(path, "rb") as fp:
        data = fp.read()
    idx = data.find(MAGIC_LE)
    while idx != -1:
        if idx + HEADER_BYTES > len(data):
            break
        if idx >= DATA_PER_CHIRP:
            _, gcount, fid, cid = struct.unpack("<4I", data[idx:idx + HEADER_BYTES])
            frames[fid] = frames.get(fid, 0) + 1
        idx = data.find(MAGIC_LE, idx + HEADER_BYTES)

    live_ids = {r["frame_id"] for r in live_rows}
    complete = sum(1 for fid in live_ids if frames.get(fid, 0) == CHIRPS_PER_FRAME)

    print(f"\n--- File verification ({size/1e6:.1f} MB on disk) ---")
    print(f"Frames logged live:            {len(live_rows)}")
    print(f"Of those, complete in file:    {complete} "
          f"({CHIRPS_PER_FRAME} chirps each)")
    ok = complete == len(live_rows)
    print("File matches live capture." if ok
          else "MISMATCH - file differs from live stats.")
    return ok


# ============================================================
# Console summary
# ============================================================
def summarize(rows, file_ok):
    dts = [r["dt_ms"] for r in rows[1:]]
    mean_dt = sum(dts) / len(dts) if dts else 0.0
    dropped = rows[-1]["dropped_total"]
    lost = rows[-1]["dca_lost"]
    miss_frames = sum(1 for r in rows if r["missing"] > 0)

    print("\n--- Summary ---")
    print(f"Frames captured:      {len(rows)}")
    if dts:
        print(f"Mean dt:              {mean_dt:.2f} ms "
              f"(target {EXPECTED_DT_MS:.0f} ms, {1000/mean_dt:.1f} fps)")
        print(f"Min / Max dt:         {min(dts):.2f} / {max(dts):.2f} ms")
    print(f"Chirps dropped:       {dropped}")
    print(f"DCA packets lost:     {lost}")
    print(f"Frames w/ missing:    {miss_frames}")

    rate_ok = dts and abs(mean_dt - EXPECTED_DT_MS) < 0.2 * EXPECTED_DT_MS
    if rate_ok and dropped == 0 and lost == 0 and miss_frames == 0 and file_ok:
        print(f"\nVERDICT: PASS - holds {EXPECTED_DT_MS:.0f} ms with zero loss, "
              "data saved and verified.")
    else:
        print("\nVERDICT: CHECK - see stats above.")


def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bin_path = os.path.join(OUT_DIR, f"capture_{stamp}.bin")
    csv_path = os.path.join(OUT_DIR, f"capture_{stamp}.csv")

    pub_q = mp.Queue(maxsize=8)
    stop_evt = mp.Event()
    disp = mp.Process(target=display_proc, args=(bin_path, pub_q, stop_evt),
                      daemon=True)
    disp.start()

    try:
        rows = capture(bin_path, pub_q)
    finally:
        stop_evt.set()
        disp.join(timeout=2.0)
        if disp.is_alive():
            disp.terminate()

    if not rows:
        print("No frames captured - nothing to write.")
        return
    write_csv(rows, csv_path)
    file_ok = verify_bin(bin_path, rows)
    summarize(rows, file_ok)


if __name__ == "__main__":
    mp.freeze_support()
    main()