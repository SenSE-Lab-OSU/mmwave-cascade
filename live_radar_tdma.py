"""
live_radar_tdma.py
Live 2-chip cascade display for TDMA (6 TX take turns).  48-channel sum.

SPEED NOTE (v2):
  The receive thread now only ASSEMBLES the frames it will actually display
  (1 in ASSEMBLE_EVERY).  For all other frames it just skims the headers to
  track loss and frame boundaries - the same light work the validator does,
  which never lagged.  It also trims its buffer every packet so memory stays
  tiny.  This is what fixes the 100+ ms lag.

RUN ORDER unchanged: start this first, then resume the R5F core in CCS.
"""

import socket
import struct
import threading
import time
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# CONFIG - must match firmware
# ============================================================
SAMPLES_PER_CHIRP = 256
NUM_RX            = 8
NUM_TX            = 6
NUM_LOOPS         = 64
CHIRPS_PER_FRAME  = NUM_LOOPS * NUM_TX     # 384

CENTER_FREQ_HZ    = 78e9
FREQ_SLOPE_MHZ_US = 1554 * 48.279e-3
SAMPLE_RATE_KSPS  = 10000
RAMP_END_US       = 28.0
IDLE_US           = 7.0

PEAK_THRESH_DB = 12.0
MAX_PEAKS      = 5
MIN_RANGE_M    = 0.5

# How often to build a full frame for display. 20 fps / 6 = ~3 shown per sec.
# Raise it if still heavy, lower it (toward 1) for a faster refresh.
ASSEMBLE_EVERY = 1

# ---- header format (must match firmware) ----
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

# ============================================================
# Derived axes
# ============================================================
C = 3e8
LAMBDA = C / CENTER_FREQ_HZ
T_ADC_US = SAMPLES_PER_CHIRP / SAMPLE_RATE_KSPS * 1e3
BANDWIDTH_HZ = FREQ_SLOPE_MHZ_US * T_ADC_US * 1e6
RANGE_RES_M = C / (2 * BANDWIDTH_HZ)
NUM_RANGE_BINS = SAMPLES_PER_CHIRP
MAX_RANGE_M = RANGE_RES_M * SAMPLES_PER_CHIRP

T_CHIRP_EFF_S = NUM_TX * (RAMP_END_US + IDLE_US) * 1e-6   # TX repeats every 6th chirp
VEL_RES_MS = LAMBDA / (2 * NUM_LOOPS * T_CHIRP_EFF_S)
MAX_VEL_MS = LAMBDA / (4 * T_CHIRP_EFF_S)

range_axis = np.arange(NUM_RANGE_BINS) * RANGE_RES_M
vel_axis = (np.arange(NUM_LOOPS) - NUM_LOOPS // 2) * VEL_RES_MS

print(f"TDMA display: {CHIRPS_PER_FRAME} chirps/frame, assembling 1 in {ASSEMBLE_EVERY}")
print(f"Range max {MAX_RANGE_M:.1f} m, res {RANGE_RES_M*100:.1f} cm | "
      f"Vel +/-{MAX_VEL_MS:.1f} m/s, res {VEL_RES_MS:.2f} m/s")


def dca_command(code, data=b""):
    return (struct.pack("<H", 0xA55A) + struct.pack("<H", code)
            + struct.pack("<H", len(data)) + data + struct.pack("<H", 0xEEAA))


def send_config_command(code):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST_IP, CONFIG_PORT)); s.settimeout(2.0)
    s.sendto(dca_command(code), (FPGA_IP, CONFIG_PORT)); s.close()


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.count = 0
        self.running = True


# ============================================================
# RX thread - light by default, assembles only displayed frames
# ============================================================
def rx_thread_fn(sh):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RECV_BUF)
    sock.bind((HOST_IP, DATA_PORT)); sock.settimeout(5.0)
    print(f"Listening on {HOST_IP}:{DATA_PORT}")

    send_config_command(CMD_START_RECORD)
    print("Streaming started. Resume the R5F core in CCS now.")

    buf = bytearray()
    scan = 0

    capture = False
    slots = None
    cur_frame = None
    got = 0
    frames_seen = 0

    prev_gcount = None
    chirps_dropped = 0
    dca_pkts = dca_lost = 0
    expected_seq = None

    prev_bt = None
    dt_hist = []

    def finalize(frame_id, got_count):
        parts = [slots[c] if slots[c] is not None else bytes(DATA_PER_CHIRP)
                 for c in range(CHIRPS_PER_FRAME)]
        frame_bytes = b"".join(parts)
        meta = {
            "frame_id": frame_id,
            "got": got_count,
            "missing": CHIRPS_PER_FRAME - got_count,
            "dropped_total": chirps_dropped,
            "dt_ms": (sum(dt_hist) / len(dt_hist)) if dt_hist else 0.0,
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

        seq = struct.unpack("<I", pkt[0:4])[0]
        dca_pkts += 1
        if expected_seq is not None and seq > expected_seq:
            dca_lost += seq - expected_seq
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

                if fid != cur_frame:
                    # frame boundary: time it (always, cheap) for the lag metric
                    t = time.perf_counter()
                    if prev_bt is not None:
                        dt_hist.append((t - prev_bt) * 1e3)
                        if len(dt_hist) > 60:
                            dt_hist.pop(0)
                    prev_bt = t
                    # finish the previous frame if we were capturing it
                    if capture and cur_frame is not None:
                        finalize(cur_frame, got)
                    # start the new frame
                    cur_frame = fid
                    frames_seen += 1
                    capture = ((frames_seen - 1) % ASSEMBLE_EVERY == 0)
                    slots = [None] * CHIRPS_PER_FRAME if capture else None
                    got = 0

                if capture and 0 <= cid < CHIRPS_PER_FRAME and slots[cid] is None:
                    slots[cid] = buf[idx - DATA_PER_CHIRP: idx]
                    got += 1

            scan = idx + HEADER_BYTES
            idx = buf.find(MAGIC_LE, scan)

        # trim everything we've already passed -> buf stays ~one chirp small
        if scan > 0:
            del buf[:scan]
            scan = 0

    send_config_command(CMD_STOP_RECORD)
    sock.close()
    print("RX thread stopped.")


# ============================================================
# Processing - TDMA reshape + 48-channel sum
# ============================================================
def parse_frame(raw_i16):
    d = raw_i16.astype(np.float32)
    adc = d[0::2] + 1j * d[1::2]
    # chirp c = loop*NUM_TX + tx -> Fortran reshape drops TX between RX and loops
    return adc.reshape((SAMPLES_PER_CHIRP, NUM_RX, NUM_TX, NUM_LOOPS), order='F')


def process(adc):
    rng = np.fft.fft(adc, axis=0)
    rd = np.fft.fftshift(np.fft.fft(rng, axis=3), axes=3)

    rd_sum = np.sum(np.abs(rd[:NUM_RANGE_BINS]), axis=(1, 2))   # sum 48 channels
    rd_db = 20 * np.log10(rd_sum + 1e-6)
    rd_db -= rd_db.max()

    rng_mag = np.abs(rng[:NUM_RANGE_BINS])
    rp = np.sum(np.mean(rng_mag, axis=3), axis=(1, 2))
    rp_db = 20 * np.log10(rp + 1e-6)
    rp_db -= rp_db.max()
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
# MAIN
# ============================================================
def main():
    sh = Shared()
    rx = threading.Thread(target=rx_thread_fn, args=(sh,), daemon=True)
    rx.start()

    plt.ion()
    fig = plt.figure(figsize=(13, 8))
    fig.suptitle("Live 2-chip Cascade Radar (TDMA, 48-channel sum)", fontsize=14)
    gs = fig.add_gridspec(2, 2, height_ratios=[3, 1])
    ax_rd = fig.add_subplot(gs[0, 0])
    ax_rp = fig.add_subplot(gs[0, 1])
    ax_st = fig.add_subplot(gs[1, :]); ax_st.axis('off')

    rd_img = ax_rd.imshow(np.zeros((NUM_RANGE_BINS, NUM_LOOPS)), aspect='auto',
                          origin='lower', cmap='viridis',
                          extent=[vel_axis[0], vel_axis[-1], 0, MAX_RANGE_M],
                          vmin=-35, vmax=0, animated=True)
    ax_rd.set_xlabel("Velocity (m/s)"); ax_rd.set_ylabel("Range (m)")
    ax_rd.set_title("Range-Doppler (48 ch)")
    fig.colorbar(rd_img, ax=ax_rd, label="dB below peak")
    peak_scatter, = ax_rd.plot([], [], 'rx', markersize=10, markeredgewidth=2, animated=True)

    rp_line, = ax_rp.plot(range_axis, np.zeros(NUM_RANGE_BINS), animated=True)
    ax_rp.set_xlabel("Range (m)"); ax_rp.set_ylabel("dB below peak")
    ax_rp.set_title("Range Profile"); ax_rp.grid(True)
    ax_rp.set_ylim(-45, 5); ax_rp.set_xlim(0, MAX_RANGE_M)

    stat = ax_st.text(0.01, 0.9, "", fontsize=10, family='monospace', va='top',
                      color='navy', transform=ax_st.transAxes, animated=True)

    fig.canvas.draw()
    bg_rd = fig.canvas.copy_from_bbox(ax_rd.bbox)
    bg_rp = fig.canvas.copy_from_bbox(ax_rp.bbox)
    bg_st = fig.canvas.copy_from_bbox(ax_st.bbox)

    last = -1
    try:
        while plt.fignum_exists(fig.number):
            with sh.lock:
                item = sh.latest
                fc = sh.count
            if item is not None and fc != last:     # RX already throttles; show every one
                frame_bytes, meta = item
                raw = np.frombuffer(frame_bytes, dtype=np.int16)
                adc = parse_frame(raw)

                # --- save one frame for TDMA validation, then keep running ---
                if not getattr(main, "_saved", False):
                    np.save("frame.npy", adc)
                    print(">>> saved frame.npy")
                    main._saved = True
                # --------------------------------------------------------------
                rp_db, rd_db = process(adc)
                peaks = find_peaks(rd_db)

                rp_line.set_ydata(rp_db)
                rd_img.set_data(rd_db)
                if peaks:
                    peak_scatter.set_data([p[1] for p in peaks], [p[0] for p in peaks])
                else:
                    peak_scatter.set_data([], [])

                complete = "OK " if meta["missing"] == 0 else "PART"
                stat.set_text(
                    f"frame {meta['frame_id']:>6} [{complete}] | "
                    f"chirps {meta['got']:>3}/{CHIRPS_PER_FRAME}  missing {meta['missing']:>3} | "
                    f"mean dt {meta['dt_ms']:5.1f} ms (per radar frame)\n"
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
