"""
live_radar_header.py
Live 2-chip cascade radar display that uses the per-chirp LVDS header.

What's different from live_radar_fast.py
----------------------------------------
The old script reassembled frames by blind byte-counting: it assumed the very
first byte was sample 0 of frame 0, then sliced every FRAME_SIZE_BYTES and hoped
alignment never drifted. A single dropped packet broke alignment permanently
(it just cleared the buffer and kept counting from wherever it landed).

Now the firmware stamps a 16-byte header after every chirp:
    [ CSIA 4096B ][ CSIB 4096B ][ magic | globalCount | frameId | chirpId ]  = 8208B
so this script:
  * locks onto the magic word to find true chirp boundaries (no assumption),
  * drops each chirp into slot[chirpId] of its frame -> data can't get misordered,
  * uses frameId to know exactly when a frame is complete,
  * detects missing chirps (gap in globalCount) and zero-fills + flags them,
  * recovers after loss by finding the next magic instead of corrupting.

The radar math (range FFT / Doppler FFT / peak find) is unchanged from the old
script -- only the *frame assembly* is now header-driven instead of byte-counted.

RUN ORDER (unchanged): start this first, then resume the R5F core in CCS.
"""

import socket
import struct
import threading
import time
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# RADAR CONFIG - must match firmware
# ============================================================
SAMPLES_PER_CHIRP  = 256
NUM_RX_PER_DEVICE  = 4
NUM_DEVICES        = 2
NUM_RX             = NUM_RX_PER_DEVICE * NUM_DEVICES   # 8
NUM_LOOPS          = 64        # TEST_NUM_LOOPS (chirps per frame) -- match firmware!
NUM_CHIRPS_IN_LOOP = 1
RX_CHANNEL         = 2         # which virtual RX to show in the plots

CENTER_FREQ_HZ   = 78e9
FREQ_SLOPE_MHZ_US = 1554 * 48.279e-3
SAMPLE_RATE_KSPS = 10000
RAMP_END_US      = 28
IDLE_US          = 7.0

PEAK_THRESH_DB = 12.0
MAX_PEAKS      = 5
MIN_RANGE_M    = 0.5
DISPLAY_EVERY  = 3            # redraw every Nth completed frame

# ============================================================
# LVDS HEADER FORMAT - must match firmware LvdsFrameHeader
# ============================================================
HEADER_MAGIC = 0xA1B2C3D4
HEADER_BYTES = 16             # magic + globalCount + frameId + chirpId (4 x uint32)
MAGIC_LE     = struct.pack("<I", HEADER_MAGIC)

BYTES_PER_SAMPLE = 4          # complex int16 (I + Q)
DATA_PER_CHIRP   = SAMPLES_PER_CHIRP * NUM_RX * BYTES_PER_SAMPLE   # 8192
BLOCK_BYTES      = DATA_PER_CHIRP + HEADER_BYTES                   # 8208
FRAME_DATA_BYTES = DATA_PER_CHIRP * NUM_LOOPS                      # one frame of pure ADC

# ============================================================
# NETWORK CONFIG (same as capture script)
# ============================================================
FPGA_IP, HOST_IP = "192.168.33.180", "192.168.33.30"
CONFIG_PORT, DATA_PORT = 4096, 4098
DCA_HEADER_BYTES = 10
SOCKET_RECV_BUF = 2 ** 26
CMD_START_RECORD, CMD_STOP_RECORD = 0x05, 0x06
MAX_BUF = 6 * BLOCK_BYTES * NUM_LOOPS

# ============================================================
# Derived (range/velocity axes) - same as old script
# ============================================================
C = 3e8
LAMBDA = C / CENTER_FREQ_HZ
T_ADC_US = SAMPLES_PER_CHIRP / SAMPLE_RATE_KSPS * 1e3
BANDWIDTH_HZ = FREQ_SLOPE_MHZ_US * T_ADC_US * 1e6
RANGE_RES_M = C / (2 * BANDWIDTH_HZ)
MAX_RANGE_M = RANGE_RES_M * SAMPLES_PER_CHIRP
T_CHIRP_S = (RAMP_END_US + IDLE_US) * 1e-6
VEL_RES_MS = LAMBDA / (2 * NUM_LOOPS * T_CHIRP_S)
MAX_VEL_MS = LAMBDA / (4 * T_CHIRP_S)

NUM_RANGE_BINS = SAMPLES_PER_CHIRP
range_axis = np.arange(NUM_RANGE_BINS) * RANGE_RES_M
vel_axis = (np.arange(NUM_LOOPS) - NUM_LOOPS // 2) * VEL_RES_MS


# ============================================================
# DCA1000 commands
# ============================================================
def dca_command(code, data=b""):
    return (struct.pack("<H", 0xA55A) + struct.pack("<H", code)
            + struct.pack("<H", len(data)) + data + struct.pack("<H", 0xEEAA))


def send_config_command(code):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST_IP, CONFIG_PORT)); s.settimeout(2.0)
    s.sendto(dca_command(code), (FPGA_IP, CONFIG_PORT)); s.close()


# ============================================================
# Shared state between RX thread and display
# ============================================================
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None        # (frame_bytes, meta)
        self.count = 0
        self.running = True


# ============================================================
# RX THREAD: header-driven frame assembly
# ============================================================
def rx_thread_fn(sh):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RECV_BUF)
    sock.bind((HOST_IP, DATA_PORT)); sock.settimeout(5.0)
    print(f"Listening on {HOST_IP}:{DATA_PORT}")

    send_config_command(CMD_START_RECORD)
    print("Streaming started. Resume the R5F core in CCS now.")

    buf = bytearray()
    scan = 0                       # relative scan position in buf
    base = 0                       # absolute offset of buf[0] (for trimming)

    # frame assembly state
    slots = [None] * NUM_LOOPS     # chirpId -> data bytes for the frame in progress
    cur_frame = None               # frameId currently being assembled
    got = 0                        # chirps placed in the current frame

    # running stats
    prev_gcount = None
    chirps_dropped = 0
    dca_pkts = dca_lost = 0
    expected_seq = None
    prev_frame_t = None

    def finalize(frame_id, got_count, t_done):
        """Assemble the 64 chirp slots (zero-filling gaps) and hand to display."""
        nonlocal prev_frame_t
        parts = [slots[c] if slots[c] is not None else bytes(DATA_PER_CHIRP)
                 for c in range(NUM_LOOPS)]
        frame_bytes = b"".join(parts)
        dt_ms = 0.0 if prev_frame_t is None else (t_done - prev_frame_t) * 1e3
        prev_frame_t = t_done
        meta = {
            "frame_id": frame_id,
            "got": got_count,
            "missing": NUM_LOOPS - got_count,
            "dropped_total": chirps_dropped,
            "dt_ms": dt_ms,
            "loss_pct": 100.0 * dca_lost / max(dca_pkts, 1),
        }
        with sh.lock:
            sh.latest = (frame_bytes, meta)
            sh.count += 1

    while sh.running:
        try:
            pkt = sock.recv(2048)
        except socket.timeout:
            print("No data 5s - waiting..."); continue
        if len(pkt) <= DCA_HEADER_BYTES:
            continue

        # DCA transport-level loss (independent cross-check)
        seq = struct.unpack("<I", pkt[0:4])[0]
        dca_pkts += 1
        if expected_seq is not None and seq > expected_seq:
            dca_lost += seq - expected_seq
        expected_seq = seq + 1

        buf.extend(pkt[DCA_HEADER_BYTES:])

        # walk every complete chirp block we can see
        idx = buf.find(MAGIC_LE, scan)
        while idx != -1:
            if idx + HEADER_BYTES > len(buf):
                break                       # partial header at the tail
            if idx >= DATA_PER_CHIRP:        # full data block precedes this magic
                data = bytes(buf[idx - DATA_PER_CHIRP: idx])
                _, gcount, fid, cid = struct.unpack("<4I", buf[idx:idx + HEADER_BYTES])

                # chirp-level loss via the never-resetting counter
                if prev_gcount is not None and gcount > prev_gcount + 1:
                    chirps_dropped += gcount - prev_gcount - 1
                prev_gcount = gcount

                # frame boundary handling
                if cur_frame is None:
                    cur_frame, slots, got = fid, [None] * NUM_LOOPS, 0
                elif fid != cur_frame:
                    finalize(cur_frame, got, time.perf_counter())
                    cur_frame, slots, got = fid, [None] * NUM_LOOPS, 0

                if 0 <= cid < NUM_LOOPS and slots[cid] is None:
                    slots[cid] = data
                    got += 1

            scan = idx + HEADER_BYTES
            idx = buf.find(MAGIC_LE, scan)
        else:
            scan = max(scan, len(buf) - (len(MAGIC_LE) - 1))

        # bound memory: keep a few chirps of tail behind the scan point
        if len(buf) > MAX_BUF:
            cut = scan - 3 * BLOCK_BYTES
            if cut > 0:
                del buf[:cut]; base += cut; scan -= cut

    send_config_command(CMD_STOP_RECORD)
    sock.close()
    print("RX thread stopped.")


# ============================================================
# Processing (unchanged from the old script)
# ============================================================
def parse_frame(raw_i16):
    d = raw_i16.astype(np.float32)
    adc = d[0::2] + 1j * d[1::2]
    adc = adc.reshape((SAMPLES_PER_CHIRP, NUM_RX, NUM_CHIRPS_IN_LOOP, NUM_LOOPS), order='F')
    return adc.reshape((SAMPLES_PER_CHIRP, NUM_RX, NUM_LOOPS), order='F')


def process(adc):
    rng = np.fft.fft(adc, axis=0)
    rd = np.fft.fftshift(np.fft.fft(rng, axis=2), axes=2)
    rp_db = 20 * np.log10(np.mean(np.abs(rng[:NUM_RANGE_BINS, RX_CHANNEL, :]), axis=1) + 1e-6)
    rd_db = 20 * np.log10(np.abs(rd[:NUM_RANGE_BINS, RX_CHANNEL, :]) + 1e-6)
    return rp_db, rd_db


def find_peaks(rd_db):
    med = np.median(rd_db)
    idx = np.argsort(rd_db.flatten())[::-1]
    peaks, used = [], []
    for i in idx[:200]:
        r, v = i // rd_db.shape[1], i % rd_db.shape[1]
        if rd_db[r, v] - med < PEAK_THRESH_DB:
            break
        if r * RANGE_RES_M < MIN_RANGE_M:
            continue
        if any(abs(r - pr) < 3 and abs(v - pv) < 3 for pr, pv in used):
            continue
        used.append((r, v))
        peaks.append((r * RANGE_RES_M, vel_axis[v], rd_db[r, v]))
        if len(peaks) >= MAX_PEAKS:
            break
    return peaks


# ============================================================
# MAIN: display
# ============================================================
def main():
    print(f"Block: {BLOCK_BYTES}B/chirp ({DATA_PER_CHIRP} data + {HEADER_BYTES} header), "
          f"{NUM_LOOPS} chirps/frame")
    sh = Shared()
    rx = threading.Thread(target=rx_thread_fn, args=(sh,), daemon=True)
    rx.start()

    plt.ion()
    fig = plt.figure(figsize=(13, 8))
    fig.suptitle("Live 2-chip Cascade Radar (header-synced)", fontsize=14)
    gs = fig.add_gridspec(2, 2, height_ratios=[3, 1])
    ax_rd = fig.add_subplot(gs[0, 0])
    ax_rp = fig.add_subplot(gs[0, 1])
    ax_st = fig.add_subplot(gs[1, :]); ax_st.axis('off')

    rd_img = ax_rd.imshow(np.zeros((NUM_RANGE_BINS, NUM_LOOPS)), aspect='auto',
                          origin='lower', cmap='viridis',
                          extent=[vel_axis[0], vel_axis[-1], 0, MAX_RANGE_M],
                          vmin=60, vmax=140, animated=True)
    ax_rd.set_xlabel("Velocity (m/s)"); ax_rd.set_ylabel("Range (m)")
    ax_rd.set_title("Range-Doppler")
    fig.colorbar(rd_img, ax=ax_rd, label="dB")
    peak_scatter, = ax_rd.plot([], [], 'rx', markersize=10, markeredgewidth=2, animated=True)

    rp_line, = ax_rp.plot(range_axis, np.zeros(NUM_RANGE_BINS), animated=True)
    ax_rp.set_xlabel("Range (m)"); ax_rp.set_ylabel("Magnitude (dB)")
    ax_rp.set_title("Range Profile"); ax_rp.grid(True)
    ax_rp.set_ylim(40, 150); ax_rp.set_xlim(0, MAX_RANGE_M)

    stat = ax_st.text(0.01, 0.9, "", fontsize=10, family='monospace', va='top',
                      color='navy', transform=ax_st.transAxes, animated=True)

    fig.canvas.draw()
    bg_rd = fig.canvas.copy_from_bbox(ax_rd.bbox)
    bg_rp = fig.canvas.copy_from_bbox(ax_rp.bbox)
    bg_st = fig.canvas.copy_from_bbox(ax_st.bbox)

    last = -1
    dt_history = []
    try:
        while plt.fignum_exists(fig.number):
            with sh.lock:
                item = sh.latest
                fc = sh.count
            if item is not None and fc != last and fc % DISPLAY_EVERY == 0:
                frame_bytes, meta = item
                raw = np.frombuffer(frame_bytes, dtype=np.int16)
                adc = parse_frame(raw)
                rp_db, rd_db = process(adc)
                peaks = find_peaks(rd_db)

                rp_line.set_ydata(rp_db)
                rd_img.set_data(rd_db)
                if peaks:
                    peak_scatter.set_data([p[1] for p in peaks], [p[0] for p in peaks])
                else:
                    peak_scatter.set_data([], [])

                dt_history.append(meta["dt_ms"])
                if len(dt_history) > 100:
                    dt_history.pop(0)
                recent = [d for d in dt_history if d > 0]
                mean_dt = np.mean(recent) if recent else 0.0

                # partial frames (a chirp was lost) are flagged, not silently shown clean
                complete = "OK " if meta["missing"] == 0 else "PART"
                stat.set_text(
                    f"frame {meta['frame_id']:>6} [{complete}] | "
                    f"chirps {meta['got']:>2}/{NUM_LOOPS}  missing {meta['missing']:>2} | "
                    f"mean dt {mean_dt:5.1f} ms\n"
                    f"chirps dropped (total) {meta['dropped_total']:>6} | "
                    f"DCA pkt loss {meta['loss_pct']:.2f}%")

                fig.canvas.restore_region(bg_rd)
                fig.canvas.restore_region(bg_rp)
                fig.canvas.restore_region(bg_st)
                ax_rd.draw_artist(rd_img)
                ax_rd.draw_artist(peak_scatter)
                ax_rp.draw_artist(rp_line)
                ax_st.draw_artist(stat)
                fig.canvas.blit(ax_rd.bbox)
                fig.canvas.blit(ax_rp.bbox)
                fig.canvas.blit(ax_st.bbox)
                fig.canvas.flush_events()
                last = fc
            else:
                plt.pause(0.005)
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        sh.running = False
        rx.join(timeout=4.0)
        print("Done.")


if __name__ == "__main__":
    main()
