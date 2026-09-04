"""
capture_beamform_ddma.py  (DDMA - 64 chirps/frame, all 6 TX simultaneous)

Capture half: byte-for-byte the trusted capture_ddma.py flow - receiver
thread, magic-scan framing, CSV log, seek verification, summary.

Display half (separate process): live beamforming of the strongest range peak,
with background subtraction. Chirp blocks are [8192 B data][16 B header]
(magic marks the header, which sits AFTER its data - per bgsub_3d.py).

Sequence:
  1. COLLECT (first BG_S seconds of frames): keep the scene empty/static. The
     display shows the raw range profile and a "collecting background" banner.
     The template is the mean DEMODULATED cube (TX x RX x SAMPLES) - for DDMA
     the raw static return varies chirp-to-chirp with the phase codes, but
     demod is linear, so subtracting the mean demod cube is exactly the
     per-chirp-cycle template. Set BG_S = 0 to skip subtraction entirely.
  2. RUN: every DISPLAY_PERIOD, read the last AVG_S s of frames (lagged
     DISPLAY_LAG frames behind live), subtract the template, then:
     range profile -> peak bin in [GATE_LO_M, GATE_HI_M] -> snapshots at the
     peak -> covariance over the batch -> covariance Bartlett on the far-field
     grid, calibrated (calibration.npz) AND uncalibrated, with confidence.

The 64-chirp DDMA code cycle is anchored on the header chirp id; frames with
gcount gaps are dropped. Geometry imports from bgsub_3d (_xs/_zs in lambda,
tx-major Dev1.TX0..Dev2.TX2 x RX0..7, AZ_SIGN on the azimuth cosine).
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

FILE_BUF_BYTES = 1 << 22   # 4 MB file buffer

# ---- live display (separate process) ----
DISPLAY_PERIOD = 1      # seconds between redraws (slow on purpose)
DISPLAY_LAG    = 16     # frames behind live for the display (write buffer)
DISPLAY_LOOPS  = 64        # chirps read per redraw: one whole DDMA frame
DYN_RANGE_DB   = 40        # color span below the peak, dB
LAYOUT_B       = False     # int16 interleave guess: flip to True if range looks wrong

# ---- beamforming display ----
ELEV_MODE   = True                 # True: fine axis = elevation; False: fine axis = azimuth
CAL_FILE    = "calibration.npz"    # 48-channel calibration (CAL 6x8 + usable mask)
GATE_LO_M, GATE_HI_M = 1.0, 5.5    # peak search window
AVG_S       = 1.0                  # covariance window (s of frames per redraw)
BG_S        = 10.0                 # background collection time (s). 0 = no subtraction.
TRACE_S     = 60                   # az/el history shown (s)
FINE_LIM, FINE_STEP     = 75.0, 0.5
COARSE_LIM, COARSE_STEP = 30.0, 1.0
TXB = [0, 56, 48, 40, 32, 24]      # slow-time bins Dev1.TX0..Dev2.TX2
FPS_NOMINAL = 1000.0 / EXPECTED_DT_MS

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
        send_config_command(CMD_STOP_RECORD)
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
# Display process - live beamforming + background subtraction
# ============================================================
def _range_axis():
    fs = ADC_RATE_KSPS * 1e3
    S  = SLOPE_MHZ_US * 1e12
    dR = C_LIGHT * fs / (2.0 * S * SAMPLES_PER_CHIRP)
    return np.arange(SAMPLES_PER_CHIRP) * dR


def _load_cal():
    z = np.load(CAL_FILE)
    cal = np.asarray(z["CAL"]).reshape(NUM_TX, NUM_RX).astype(np.complex64)
    usable = (np.asarray(z["usable"]).reshape(NUM_TX, NUM_RX)
              if "usable" in z.files else np.ones((NUM_TX, NUM_RX), bool))
    return cal, usable


def _build_grid():
    """Far-field conjugated steering dictionary from bgsub_3d geometry."""
    import bgsub_3d as bg
    bg.configure_geometry(ELEV_MODE)
    xs = np.asarray(bg._xs, np.float64).ravel()
    zs = np.asarray(bg._zs, np.float64).ravel()
    az_sign = float(bg.AZ_SIGN)
    nv = NUM_TX * NUM_RX
    assert xs.size == nv and zs.size == nv, "geometry size != 48 virtual channels"
    if ELEV_MODE:
        az_g = np.arange(-COARSE_LIM, COARSE_LIM + 1e-9, COARSE_STEP)
        el_g = np.arange(-FINE_LIM,   FINE_LIM   + 1e-9, FINE_STEP)
    else:
        az_g = np.arange(-FINE_LIM,   FINE_LIM   + 1e-9, FINE_STEP)
        el_g = np.arange(-COARSE_LIM, COARSE_LIM + 1e-9, COARSE_STEP)
    azm, elm = np.meshgrid(np.radians(az_g), np.radians(el_g))
    u = az_sign * np.sin(azm) * np.cos(elm)
    w = np.sin(elm)
    a_c = np.exp(-2j * np.pi * (u[..., None] * xs + w[..., None] * zs))
    return az_g, el_g, a_c.reshape(-1, nv).astype(np.complex64)


def _bartlett(R, az_g, el_g, a_c):
    P = np.einsum("mi,ij,mj->m", a_c, R, a_c.conj()).real.reshape(len(el_g), len(az_g))
    k = np.unravel_index(np.argmax(P), P.shape)
    conf = 10.0 * np.log10(P[k] / np.median(P))
    return P, float(az_g[k[1]]), float(el_g[k[0]]), float(conf)


def _read_frames(fr, start_off, end_off, layout_b):
    """Read [start_off, end_off), magic-align, return complete DDMA frames as
    (n, CHIRPS, RX, SAMPLES) complex64.

    Block layout is [DATA_PER_CHIRP data][16 B header] - the magic marks the
    header, which sits AFTER its data (bgsub_3d.py), so each chirp's data is
    the 8192 bytes BEFORE its magic. Frames are anchored on the header chirp
    id (cid % CHIRPS_PER_FRAME == 0) so the 64-chirp DDMA phase code cycle
    starts at chirp 0 - required for inter-TX phase to mean anything after
    demod. Blocks with non-consecutive gcount are discarded."""
    start = max(0, start_off)
    fr.seek(start)
    raw = fr.read(end_off - start)
    if len(raw) < FRAME_BYTES:
        return None
    frames, chirps, last_g = [], [], None
    idx = raw.find(MAGIC_LE, DATA_PER_CHIRP)
    while idx != -1 and idx + HEADER_BYTES <= len(raw):
        _, gcount, fid, cid = struct.unpack("<4I", raw[idx:idx + HEADER_BYTES])
        data = raw[idx - DATA_PER_CHIRP: idx]
        if last_g is not None and gcount != last_g + 1:
            chirps = []                                   # gap -> restart frame
        last_g = gcount
        if cid % CHIRPS_PER_FRAME == 0:
            chirps = [data]
        elif chirps:
            chirps.append(data)
        if len(chirps) == CHIRPS_PER_FRAME:
            frames.append(b"".join(chirps))
            chirps = []
        idx = raw.find(MAGIC_LE, idx + HEADER_BYTES + DATA_PER_CHIRP - len(MAGIC_LE))
    if not frames:
        return None
    m = np.frombuffer(b"".join(frames), np.uint8).reshape(len(frames), CHIRPS_PER_FRAME, DATA_PER_CHIRP)
    iq = np.ascontiguousarray(m).view(np.int16).astype(np.float32)
    c = iq[..., 0::2] + 1j * iq[..., 1::2]
    if not layout_b:
        cube = c.reshape(len(frames), CHIRPS_PER_FRAME, NUM_RX, SAMPLES_PER_CHIRP)
    else:
        cube = c.reshape(len(frames), CHIRPS_PER_FRAME, SAMPLES_PER_CHIRP, NUM_RX).transpose(0, 1, 3, 2)
    return cube


def _demod(cube, rwin):
    """(n, CHIRPS, RX, SAMPLES) -> (n, TX, RX, SAMPLES): range FFT + DDMA demod."""
    rng = np.fft.fft(cube * rwin, axis=-1)
    return np.fft.fft(rng, axis=1)[:, TXB]


def display_proc(bin_path, pub_q, stop_evt):
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"display: matplotlib unavailable ({e}); capturing without a live view.")
        return
    try:
        az_g, el_g, a_c = _build_grid()
        cal, usable = _load_cal()
    except Exception as e:
        print(f"display: beamforming setup failed ({e}); capturing without a live view.")
        return

    while not os.path.exists(bin_path):
        if stop_evt.is_set():
            return
        time.sleep(0.1)

    r = _range_axis()
    gate = (r >= GATE_LO_M) & (r <= GATE_HI_M)
    rwin = np.hanning(SAMPLES_PER_CHIRP).astype(np.float32)
    nv = NUM_TX * NUM_RX
    avg_frames = max(2, int(round(AVG_S * FPS_NOMINAL)))
    bg_target = int(round(BG_S * FPS_NOMINAL))

    # background state
    bg_sum = np.zeros((NUM_TX, NUM_RX, SAMPLES_PER_CHIRP), np.complex128)
    bg_n = 0
    bg_next_off = 0                      # next unread offset during collection
    template = None if bg_target > 0 else 0.0

    fr = open(bin_path, "rb")
    plt.ion()
    fig = plt.figure(figsize=(13, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.1])
    ax1, ax2, ax3 = (fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]),
                     fig.add_subplot(gs[1, :]))

    (ln,) = ax1.plot(r, np.zeros_like(r), lw=0.8)
    (pk,) = ax1.plot([], [], "rv")
    ax1.axvspan(GATE_LO_M, GATE_HI_M, color="0.9")
    ax1.set_xlabel("range (m)"); ax1.set_ylabel("dB"); ax1.set_title("range profile")

    img = ax2.imshow(np.zeros((len(el_g), len(az_g)), np.float32),
                     aspect="auto", origin="lower", cmap="viridis",
                     extent=[az_g[0], az_g[-1], el_g[0], el_g[-1]],
                     vmin=-25, vmax=0)
    (mk_c,) = ax2.plot([], [], "r+", ms=12)
    (mk_u,) = ax2.plot([], [], "wx", ms=8)
    ax2.set_xlabel("azimuth (deg)"); ax2.set_ylabel("elevation (deg)")
    fig.colorbar(img, ax=ax2, label="dB")
    banner = fig.text(0.5, 0.965, "", ha="center", va="top", fontsize=13,
                      color="tab:red", fontweight="bold")

    hist_t = deque(); hist = {k: deque() for k in ("az_c", "el_c", "az_u", "el_u")}
    lines = {}
    for key, sty, lab in (("az_c", "-", "az cal"), ("el_c", "-", "el cal"),
                          ("az_u", "--", "az uncal"), ("el_u", "--", "el uncal")):
        (lines[key],) = ax3.plot([], [], sty, lw=1.2, label=lab)
    ax3.legend(loc="upper left", ncol=4); ax3.grid(alpha=0.3)
    ax3.set_xlabel("s"); ax3.set_ylabel("deg")
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    t0 = time.perf_counter()

    try:
        while not stop_evt.is_set():
            if not plt.fignum_exists(fig.number):
                break
            item = None
            try:
                while True:
                    item = pub_q.get_nowait()          # newest frame only
            except queue.Empty:
                pass
            if item is None:
                plt.pause(DISPLAY_PERIOD)
                continue
            _fid, off, _got = item
            end_off = off - DISPLAY_LAG * FRAME_BYTES  # flushed region only
            if end_off < 2 * FRAME_BYTES:
                banner.set_text("waiting for frames...")
                plt.pause(DISPLAY_PERIOD)
                continue

            # ---------------- background collection ----------------
            if template is None:
                cube = _read_frames(fr, bg_next_off, end_off, LAYOUT_B)
                bg_next_off = max(bg_next_off, end_off - FRAME_BYTES)  # keep partial tail
                if cube is not None:
                    D = _demod(cube, rwin)
                    bg_sum += D.sum(axis=0)
                    bg_n += len(D)
                    prof = (np.abs(D) ** 2).mean(axis=(0, 1, 2))
                    ln.set_ydata(10.0 * np.log10(prof + 1e-12))
                    ax1.relim(); ax1.autoscale_view(scalex=False)
                    ax1.set_title("range profile (RAW - collecting background)")
                banner.set_text(f"COLLECTING BACKGROUND  {bg_n / FPS_NOMINAL:.1f} / "
                                f"{BG_S:.0f} s - keep the scene empty/static")
                if bg_n >= bg_target:
                    template = (bg_sum / max(bg_n, 1)).astype(np.complex64)
                    banner.set_text(f"background applied ({bg_n} frames)")
                    banner.set_color("tab:green")
                    print(f"display: background template from {bg_n} frames - subtracting")
                plt.pause(DISPLAY_PERIOD)
                continue

            # ---------------- run: subtract + beamform ----------------
            cube = _read_frames(fr, end_off - avg_frames * FRAME_BYTES - 2 * BLOCK_BYTES,
                                end_off, LAYOUT_B)
            if cube is None:
                plt.pause(DISPLAY_PERIOD)
                continue
            D = _demod(cube, rwin)
            if np.isscalar(template):
                Dc = D
            else:
                Dc = D - template[None]

            prof = (np.abs(Dc) ** 2).mean(axis=(0, 1, 2))
            pbin = int(np.argmax(np.where(gate, prof, -np.inf)))

            S = Dc[:, :, :, pbin]                                  # [n,6,8]
            Xc = (S * cal * usable).reshape(len(S), nv)
            Xu = (S * usable).reshape(len(S), nv)
            Pc, az_c, el_c, cc = _bartlett((Xc.T @ Xc.conj()) / len(S), az_g, el_g, a_c)
            _,  az_u, el_u, cu = _bartlett((Xu.T @ Xu.conj()) / len(S), az_g, el_g, a_c)

            tnow = time.perf_counter() - t0
            hist_t.append(tnow)
            for k, v in zip(hist, (az_c, el_c, az_u, el_u)):
                hist[k].append(v)
            while hist_t and hist_t[0] < tnow - TRACE_S:
                hist_t.popleft()
                for k in hist:
                    hist[k].popleft()

            pdb = 10.0 * np.log10(prof + 1e-12)
            ln.set_ydata(pdb); pk.set_data([r[pbin]], [pdb[pbin]])
            ax1.relim(); ax1.autoscale_view(scalex=False)
            bg_lab = "bg-subtracted" if not np.isscalar(template) else "raw"
            ax1.set_title(f"range profile ({bg_lab}) - peak {r[pbin]:.2f} m ({len(S)} frames)")

            img.set_data(10.0 * np.log10(Pc / Pc.max() + 1e-12))
            mk_c.set_data([az_c], [el_c]); mk_u.set_data([az_u], [el_u])
            ax2.set_title(f"cal az {az_c:+.1f} el {el_c:+.1f} ({cc:.1f} dB) | "
                          f"uncal az {az_u:+.1f} el {el_u:+.1f} ({cu:.1f} dB)")

            ht = np.fromiter(hist_t, float)
            for k in lines:
                lines[k].set_data(ht, np.fromiter(hist[k], float))
            ax3.relim(); ax3.autoscale_view()
            plt.pause(DISPLAY_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        fr.close()
        try:
            plt.ioff(); plt.close(fig)
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