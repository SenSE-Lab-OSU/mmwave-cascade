"""
capture_ddma.py  (DDMA - 64 chirps/frame, all 6 TX simultaneous, long chirp)

Capture half: unchanged, runs in its own process exactly like the trusted
capture_and_save.py. Only adds a non-blocking offset handoff to the display.

Display half (separate process): the v4 range-profile line is back, plus a
small range-Doppler map under it. To keep capture loss-free it reads only the
first DISPLAY_LOOPS loops of each frame (a small contiguous slice, ~0.8 MB)
instead of the whole 3 MB frame, and builds both views from that slice.

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

from dca1000 import DCA1000, DCA1000Error

try:
    import serial          # pyserial - radar go/stop over the XDS110 UART
except ImportError:
    serial = None

# ============================================================
# CONFIG - must match firmware
# ============================================================
SAMPLES_PER_CHIRP = 256
NUM_RX            = 8
NUM_TX            = 6
CHIRPS_PER_FRAME  = 64                          # DDMA: 8 loops x 8 chirp-RAM entries

EXPECTED_DT_MS = 1000.0 / 120.0   # 8.333 ms (120 fps DDMA)
RUN_SECONDS    = 1000
OUT_DIR        = "."

# ---- chirp block format (must match firmware) ----
HEADER_MAGIC = 0xA1B2C3D4
HEADER_BYTES = 16
MAGIC_LE     = struct.pack("<I", HEADER_MAGIC)
BYTES_PER_SAMPLE = 4
DATA_PER_CHIRP   = SAMPLES_PER_CHIRP * NUM_RX * BYTES_PER_SAMPLE   # 8192
BLOCK_BYTES      = DATA_PER_CHIRP + HEADER_BYTES                   # 8208
FRAME_BYTES      = CHIRPS_PER_FRAME * BLOCK_BYTES                  # 525,312

# ---- network ----
FPGA_IP, HOST_IP = "192.168.33.180", "192.168.33.30"
CONFIG_PORT, DATA_PORT = 4096, 4098
DCA_HEADER_BYTES = 10
SOCKET_RECV_BUF = 2 ** 26
CMD_START_RECORD, CMD_STOP_RECORD = 0x05, 0x06
DCA_PACKET_DELAY_US = 5          # AM273X_Capture.json packetDelay_us

# ---- radar control (firmware waits at READY for 'g'; 's' stops) ----
import sys as _sys
RADAR_PORT = "COM5" if _sys.platform.startswith("win") else "/dev/ttyACM0"
RADAR_BAUD = 115200
USE_RADAR_SERIAL = True          # False = old CCS workflow (you press resume)

FILE_BUF_BYTES = 1 << 22   # 4 MB file buffer

# ---- live display (separate process) ----
DISPLAY_PERIOD = 1      # seconds between redraws (slow on purpose)
DISPLAY_LAG    = 16     # frames behind live for the display (write buffer)
DISPLAY_LOOPS  = 64        # chirps read per redraw: one whole DDMA frame
DYN_RANGE_DB   = 40        # color span below the peak, dB
LAYOUT_B       = False     # int16 interleave guess: flip to True if range looks wrong

# ---- axis calibration (from rlProfileCfg) ----
# Long-chirp profile.  dR = c*fs/(2*S*N) is invariant under the k=4 stretch
# (fs and S both scaled by 1/4), so the range axis is unchanged; the velocity
# axis is not -- CHIRP_PERIOD_US drives it and must track the firmware.
C_LIGHT         = 299792458.0
ADC_RATE_KSPS   = 2500.0      # digOutSampleRate      (short chirp: 10000)
SLOPE_MHZ_US    = 37.5156     # freqSlopeConst = 777  (short chirp: 150.06)
START_FREQ_GHZ  = 76.0        # startFreqConst
CHIRP_PERIOD_US = 112.0       # idle 7 + rampEnd 105  (short chirp: 35.0)
# Scheme B (block TDM) long chirp: CHIRP_PERIOD_US = 125.0 (idle 20 + rampEnd 105)


def dca_command(code, data=b""):
    return (struct.pack("<H", 0xA55A) + struct.pack("<H", code)
            + struct.pack("<H", len(data)) + data + struct.pack("<H", 0xEEAA))


def send_config_command(code):
    """Legacy fire-and-forget (kept for reference; capture() uses DCA1000)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST_IP, CONFIG_PORT)); s.settimeout(2.0)
    s.sendto(dca_command(code), (FPGA_IP, CONFIG_PORT)); s.close()


class RadarLink:
    """Minimal go/stop over the radar's UART. Prints what the board answers."""
    def __init__(self, port=RADAR_PORT, baud=RADAR_BAUD):
        if serial is None:
            raise RuntimeError("pyserial not installed: pip install pyserial")
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

    # DCA1000: what start_capture.bat used to do, every run (idempotent).
    dca = DCA1000(FPGA_IP, HOST_IP, CONFIG_PORT)
    dca.configure(packet_delay_us=DCA_PACKET_DELAY_US)
    dca.start_record()

    radar = None
    if USE_RADAR_SERIAL:
        radar = RadarLink()
        print(f"Radar: sending 'g' on {RADAR_PORT}")
        radar.go()
    else:
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

    # contiguity state
    prev_frame_offset = None
    prev_frame_id     = None
    bad_off_steps     = 0      # frames whose byte distance != FRAME_BYTES
    bad_fid_steps     = 0      # frames whose header frame_id did not advance by 1
    bytes_missing     = 0      # cumulative byte shortfall in the stream
    fid_static        = True   # header frame_id never changed -> fallback in use

    f = open(bin_path, "wb", buffering=FILE_BUF_BYTES)

    def finalize(frame_id, got_count):
        nonlocal prev_frame_offset, prev_frame_id
        nonlocal bad_off_steps, bad_fid_steps, bytes_missing

        dt = (time.perf_counter() - prev_bt) * 1e3 if prev_bt is not None else 0.0

        # Byte distance between consecutive frame starts. The .bin is a raw
        # append of the UDP payloads, so any packet the link drops shortens
        # this by exactly the lost byte count. This is the only check that
        # sees DCA-side loss; dca_lost counts packets and cannot be converted
        # to bytes, and got/missing count headers that did arrive.
        off_step = (frame_offset - prev_frame_offset
                    if prev_frame_offset is not None else None)
        if off_step is not None and off_step != FRAME_BYTES:
            bad_off_steps += 1
            if off_step < FRAME_BYTES:
                bytes_missing += FRAME_BYTES - off_step

        # Frame counter continuity. A frame the radar emitted but that never
        # reached the host leaves the byte distance intact (its bytes were
        # never written) and shows up only here.
        fid_step = (frame_id - prev_frame_id
                    if prev_frame_id is not None else None)
        if fid_step is not None and fid_step != 1:
            bad_fid_steps += 1

        prev_frame_offset = frame_offset
        prev_frame_id     = frame_id

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
            "off_step": off_step,
            "fid_step": fid_step,
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

                    # Close a frame on a header frame_id change OR after
                    # CHIRPS_PER_FRAME chirps, whichever comes first. The
                    # second condition is the fallback: if the firmware leaves
                    # frame_id at zero the first condition never fires and the
                    # whole run collapses into one unterminated frame.
                    if fid != cur_frame or got >= CHIRPS_PER_FRAME:
                        if fid != cur_frame and cur_frame is not None:
                            fid_static = False
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
        f.close()

    print(f"Capture done: {len(rows)} complete frames recorded.")
    print(f"Peak queue depth: {max_qlen} packets "
          "(how far disk lagged the network - small is good)")
    stats = {
        "bad_off_steps": bad_off_steps,
        "bad_fid_steps": bad_fid_steps,
        "bytes_missing": bytes_missing,
        "fid_static": fid_static,
        "max_qlen": max_qlen,
    }
    return rows, stats


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
    pri = CHIRP_PERIOD_US * 1e-6                 # DDMA: PRI = one chirp; 6 TX tones appear as replicas
    vmax = lam / (4.0 * pri)
    return np.linspace(-vmax, vmax, n_loops, endpoint=False)


def _process(raw, n_loops, layout_b, rwin, dwin):
    """Slice of the frame -> (range_profile, range_doppler_map)."""
    nch = n_loops                                # DDMA: chirps have no TX identity
    m = np.frombuffer(raw, dtype=np.uint8).reshape(nch, BLOCK_BYTES)
    data = np.ascontiguousarray(m[:, :DATA_PER_CHIRP])      # strip 16-byte headers
    iq = data.view(np.int16).astype(np.float32)             # (nch, 4096)
    c = iq[:, 0::2] + 1j * iq[:, 1::2]                       # (nch, 2048)
    if not layout_b:
        c = c.reshape(nch, NUM_RX, SAMPLES_PER_CHIRP)                    # LAYOUT A
    else:
        c = c.reshape(nch, SAMPLES_PER_CHIRP, NUM_RX).transpose(0, 2, 1) # LAYOUT B

    c = c.reshape(n_loops, 1, NUM_RX, SAMPLES_PER_CHIRP)       # DDMA: keep 4-D shape, no TX axis
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
    need = DISPLAY_LOOPS * BLOCK_BYTES

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
    ax2.set_title("range vs slow-time freq (6 DDMA tones per target)")
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
            if got < DISPLAY_LOOPS:                 # need one whole frame
                plt.pause(DISPLAY_PERIOD)
                continue

            # Read a frame DISPLAY_LAG frames behind the newest. 64-chirp
            # frames (0.5 MB) are smaller than the 4 MB write buffer, so the
            # newest frame is never on disk yet; 16 frames back (8.4 MB) is.
            off -= DISPLAY_LAG * FRAME_BYTES
            if off < 0:
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
                    "chirps_dropped_total", "dca_pkts", "dca_lost", "bin_offset",
                    "off_step", "fid_step"])
        for r in rows:
            ts = datetime.datetime.fromtimestamp(r["wall_time"]).strftime(
                "%Y-%m-%d %H:%M:%S.%f")[:-3]
            w.writerow([r["frame_id"], ts, f"{r['dt_ms']:.2f}", r["got"],
                        r["missing"], r["dropped_total"], r["dca_pkts"],
                        r["dca_lost"], r["bin_offset"],
                        "" if r["off_step"] is None else r["off_step"],
                        "" if r["fid_step"] is None else r["fid_step"]])
    print(f"Wrote {path}")


# ============================================================
# Verify file on disk matches live stats
# ============================================================
def verify_bin(path, live_rows):
    """Seek-verify each logged frame instead of scanning the whole file.

    The old version did fp.read() on the entire .bin; at 20 fps a 1000 s run
    is ~63 GB and that call is not survivable. Two seeks per frame confirm the
    same thing more strictly: that the first and last chirp headers of the
    frame sit at exactly the offsets the byte layout predicts.
    """
    size = os.path.getsize(path)
    good = bad_magic = bad_fid = short = 0

    with open(path, "rb") as fp:
        for r in live_rows:
            off = r["bin_offset"]
            if off is None or r["got"] != CHIRPS_PER_FRAME:
                short += 1
                continue
            ok = True
            for k in (0, CHIRPS_PER_FRAME - 1):
                fp.seek(off + k * BLOCK_BYTES + DATA_PER_CHIRP)
                hdr = fp.read(HEADER_BYTES)
                if len(hdr) < HEADER_BYTES:
                    ok = False
                    short += 1
                    break
                magic, _gc, fid, _cid = struct.unpack("<4I", hdr)
                if magic != HEADER_MAGIC:
                    ok = False
                    bad_magic += 1
                    break
                if fid != r["frame_id"]:
                    ok = False
                    bad_fid += 1
                    break
            if ok:
                good += 1

    expected = len(live_rows) * FRAME_BYTES
    print(f"\n--- File verification ({size/1e6:.1f} MB on disk) ---")
    print(f"Frames logged live:            {len(live_rows)}")
    print(f"Verified at predicted offset:  {good}")
    if bad_magic:
        print(f"  header magic wrong:          {bad_magic}  (byte stream shifted)")
    if bad_fid:
        print(f"  frame_id mismatch:           {bad_fid}    (frames interleaved/shifted)")
    if short:
        print(f"  incomplete or truncated:     {short}")
    print(f"File size vs {len(live_rows)} x {FRAME_BYTES} B: "
          f"{size - expected:+d} B")

    ok = (good == len(live_rows))
    print("File matches live capture." if ok
          else "MISMATCH - file differs from live stats.")
    return ok


# ============================================================
# Console summary
# ============================================================
def summarize(rows, stats, file_ok):
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

    print("\n--- Stream contiguity ---")
    print(f"Bad byte steps:       {stats['bad_off_steps']} "
          f"(frame starts not exactly {FRAME_BYTES} B apart)")
    print(f"Bytes missing:        {stats['bytes_missing']}")
    if stats["fid_static"]:
        print("Frame-id continuity:  UNUSABLE - header frame_id never changed; "
              "frames were closed by the 384-chirp fallback.")
        fid_ok = True
    else:
        print(f"Bad frame-id steps:   {stats['bad_fid_steps']} "
              "(frame_id did not advance by 1)")
        fid_ok = stats["bad_fid_steps"] == 0

    rate_ok = dts and abs(mean_dt - EXPECTED_DT_MS) < 0.2 * EXPECTED_DT_MS
    if (rate_ok and dropped == 0 and lost == 0 and miss_frames == 0
            and stats["bad_off_steps"] == 0 and fid_ok and file_ok):
        print(f"\nVERDICT: PASS - holds {EXPECTED_DT_MS:.0f} ms with zero loss, "
              "byte stream contiguous, data saved and verified.")
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
        rows, stats = capture(bin_path, pub_q)
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
    summarize(rows, stats, file_ok)


if __name__ == "__main__":
    mp.freeze_support()
    main()