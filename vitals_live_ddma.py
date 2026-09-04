"""
vitals_live_ddma.py - live chest-phase display for the DDMA firmware.

Capture is reused unchanged from capture_ddma.py (short chirp, 120 fps,
64 chirps/frame). Nothing is kept: the .bin is the transport between the
capture process and the display, and it is deleted on exit.

SPEED: the per-frame work is a single dot product, not two FFTs. Once the
chest bin is locked, the range FFT + slow-time FFT + TX gather + 48-channel
sum all collapse into one fixed weight vector K, so each frame costs one
real dot over the raw int16 buffer (~0.3 ms instead of ~14 ms). The full
range profile for panel 1 is computed only on redraw, from the newest frame.

The chest bin is chosen once, from the mean range profile over the first
BIN_LOCK_S seconds inside [CHEST_MIN_M, CHEST_MAX_M], then held for the run.
Press 'r' in the window to re-lock it from the next BIN_LOCK_S seconds.

Panels:
  1  range profile |Z|, chest bin marked
  2  phasor: raw complex chest vector with a short trail
  3  DACM displacement (mm, mean-removed), scrolling window
  4  FFT of panel 3 over the last FFT_WINDOW_S, heart-band peak -> bpm
"""

import os
import queue
import time
import datetime
import multiprocessing as mp
from collections import deque

import numpy as np

from capture_ddma import (
    capture,
    SAMPLES_PER_CHIRP, NUM_RX, NUM_TX, CHIRPS_PER_FRAME,
    BLOCK_BYTES, DATA_PER_CHIRP,
    C_LIGHT, ADC_RATE_KSPS, SLOPE_MHZ_US, START_FREQ_GHZ, EXPECTED_DT_MS,
)

# ============================================================
# TUNABLES
# ============================================================
BIN_LOCK_S     = 3.0         # seconds of profile averaging before the bin locks
CHEST_MIN_M    = 0.5         # search window for the chest bin
CHEST_MAX_M    = 3.0
PHASE_WINDOW_S = 30.0        # scrolling window in panel 3 (s)
FFT_WINDOW_S   = 15.0        # heart FFT window (s); resolution = 1/this
HP_HZ          = 0.8         # highpass applied BEFORE the heart FFT (as in the notebooks)
HEART_LO_HZ    = 0.8         # heart band for the peak readout
HEART_HI_HZ    = 2.2
SPEC_VIEW_HZ   = 3.0         # panel 4 x limit
TRAIL_LEN      = 60          # phasor trail (frames)
REDRAW_PERIOD  = 0.1         # seconds between redraws
MAX_BACKLOG    = 600         # frames; older pending frames are dropped
MAX_STEP_RAD   = 1.5         # per-frame DACM step ceiling (~0.5 mm at 120 fps)
PLOT_MAX_PTS   = 1200        # decimation cap for the scrolling trace
PUB_Q_MAX      = 256
RANGE_VIEW_M   = 5.0         # panel 1 x limit

# ---- colours ----
C_PROFILE, C_CHEST = "#1f9e8f", "#d62728"          # teal profile, red chest marker
C_PHASOR, C_TRAIL  = "#ff7f0e", "#9467bd"          # orange vector, purple trail
C_RAW              = "#2ca02c"                      # green raw DACM
C_HEART            = "#1f77b4"                      # blue heart spectrum

# ---- DDMA constants (measured mapping, see ddma_mapping_check) ----
N_DDMA   = 8
TX_BINS  = [0, 56, 48, 40, 32, 24]        # TX m -> slow-time bin
NUM_VIRT = NUM_TX * NUM_RX

FRAME_PERIOD_S = EXPECTED_DT_MS / 1000.0
FRAME_BYTES    = CHIRPS_PER_FRAME * BLOCK_BYTES
F_CENTER       = (START_FREQ_GHZ * 1e9
                  + SLOPE_MHZ_US * 1e12 * (1.5e-6 + SAMPLES_PER_CHIRP / (ADC_RATE_KSPS * 1e3) / 2))
LAMBDA         = C_LIGHT / F_CENTER
MM_PER_RAD     = -LAMBDA / (4 * np.pi) * 1e3   # increasing phase = toward radar


def range_axis():
    fs = ADC_RATE_KSPS * 1e3
    S = SLOPE_MHZ_US * 1e12
    dR = C_LIGHT * fs / (2.0 * S * SAMPLES_PER_CHIRP)
    return np.arange(SAMPLES_PER_CHIRP) * dR


def make_weights(chest_bin, rwin):
    """Collapse (range FFT at one bin) x (slow-time FFT at the 6 TX bins) x
    (sum over 48 channels) into two real weight vectors over the raw int16
    frame buffer.  Z = (x . a) + 1j*(x . b) with x the frame as float32."""
    n = np.arange(SAMPLES_PER_CHIRP)
    wbin = rwin * np.exp(-2j * np.pi * chest_bin * n / SAMPLES_PER_CHIRP)
    wsum = np.exp(-2j * np.pi * np.outer(np.arange(CHIRPS_PER_FRAME), TX_BINS)
                  / CHIRPS_PER_FRAME).sum(axis=1)          # sum over the 6 TX
    K = np.outer(np.repeat(wsum, NUM_RX), wbin).ravel()    # [chirp*rx*sample]
    a = np.empty(K.size * 2, np.float32); a[0::2] = K.real; a[1::2] = -K.imag
    b = np.empty(K.size * 2, np.float32); b[0::2] = K.imag; b[1::2] = K.real
    return a, b


def frame_to_float(raw):
    """Raw frame bytes -> float32 view of the interleaved I/Q payload."""
    m = np.frombuffer(raw, dtype=np.uint8).reshape(CHIRPS_PER_FRAME, BLOCK_BYTES)
    return np.ascontiguousarray(m[:, :DATA_PER_CHIRP]).view(np.int16).astype(np.float32).ravel()


def chest_value(x, wa, wb):
    """One frame -> the complex chest-bin value.  Two dots, no FFT."""
    return complex(x @ wa, x @ wb)


_WSUM = np.exp(-2j * np.pi * np.outer(np.arange(CHIRPS_PER_FRAME), TX_BINS)
               / CHIRPS_PER_FRAME).sum(axis=1).astype(np.complex64)


def full_profile(raw, rwin):
    """Complex range profile over all bins (panel 1 only).

    The 6-TX gather is a fixed linear combination over chirps, so collapse the
    chirp axis FIRST and range-FFT only the 8 surviving RX rows: 8 FFTs per
    frame instead of 512 (10x faster, same answer to 1e-6).
    """
    m = np.frombuffer(raw, dtype=np.uint8).reshape(CHIRPS_PER_FRAME, BLOCK_BYTES)
    iq = np.ascontiguousarray(m[:, :DATA_PER_CHIRP]).view(np.int16).astype(np.float32)
    c = (iq[:, 0::2] + 1j * iq[:, 1::2]).astype(np.complex64).reshape(
        CHIRPS_PER_FRAME, NUM_RX * SAMPLES_PER_CHIRP)
    g = (_WSUM @ c).reshape(NUM_RX, SAMPLES_PER_CHIRP)
    return np.fft.fft(g * rwin[None, :], axis=1).sum(axis=0)


def dacm(z):
    """Extended DACM (Wang 2014): differentiate-cross-multiply, then accumulate."""
    n = len(z)
    if n < 2:
        return np.zeros(n)
    I, Q = z.real.astype(float), z.imag.astype(float)
    dI, dQ = np.diff(I), np.diff(Q)
    w = (I[1:] * dQ - dI * Q[1:]) / (I[1:] ** 2 + Q[1:] ** 2 + 1e-12)
    return np.concatenate([[0.0], np.cumsum(w)])


def make_highpass(fs, fc):
    """Build the highpass ONCE, at startup.

    scipy.signal costs ~1.7 s to import the first time. Doing it lazily inside
    the redraw froze the display for two seconds the moment the chest bin
    locked; importing and warming it here moves that cost to startup.
    """
    try:
        from scipy.signal import butter, sosfiltfilt
        sos = butter(4, fc / (fs / 2), "highpass", output="sos")
        sosfiltfilt(sos, np.zeros(512))                     # warm the code paths
        return lambda x: sosfiltfilt(sos, x) if len(x) > 30 else x - x.mean()
    except Exception as e:
        print(f"scipy unavailable ({e}); heart FFT uses mean removal only")
        return lambda x: x - np.mean(x)


def spectrum(x, fs):
    x = (np.asarray(x, float) - np.mean(x)) * np.hanning(len(x))
    X = np.abs(np.fft.rfft(x))
    return np.fft.rfftfreq(len(x), 1.0 / fs), X


def band_peak(f, X, lo, hi):
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return np.nan, np.nan
    idx = np.where(m)[0]
    k = idx[np.argmax(X[idx])]
    if 0 < k < len(X) - 1:
        a, b, c = X[k - 1], X[k], X[k + 1]
        den = a - 2 * b + c
        d = 0.5 * (a - c) / den if den != 0 else 0.0
    else:
        d = 0.0
    df = f[1] - f[0] if len(f) > 1 else 0.0
    return f[k] + d * df, X[k]


# ============================================================
# Display process
# ============================================================
def display_proc(bin_path, pub_q, stop_evt):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
    except Exception as e:
        print(f"display: matplotlib unavailable ({e}); capturing without a view.")
        return

    while not os.path.exists(bin_path):
        if stop_evt.is_set():
            return
        time.sleep(0.1)

    r = range_axis()
    view_mask = r <= RANGE_VIEW_M
    rwin = np.hanning(SAMPLES_PER_CHIRP).astype(np.float32)
    fs = 1.0 / FRAME_PERIOD_S
    NW = int(max(PHASE_WINDOW_S, FFT_WINDOW_S) * fs) + 1
    NFFT = max(64, int(FFT_WINDOW_S * fs))
    N_LOCK = max(1, int(BIN_LOCK_S * fs))
    hp_filter = make_highpass(fs, HP_HZ)                    # imports scipy now, not mid-run
    np.fft.rfft(np.zeros(NFFT))                             # warm the FFT path too

    st = {"chest": None, "wa": None, "wb": None, "n": 0, "nacc": 0,
          "acc": np.zeros(SAMPLES_PER_CHIRP), "prof": np.zeros(SAMPLES_PER_CHIRP),
          "last_raw": None, "prev_z": None, "phase": 0.0, "drop": 0, "clip": 0}
    buf_n = deque(maxlen=NW)          # frame index (uniform time base)
    buf_z = deque(maxlen=NW)          # complex chest value
    buf_d = deque(maxlen=NW)          # DACM displacement, mm (accumulated live)

    # ---- figure ----
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    ax1, ax2, ax3, ax4 = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]

    (ln,) = ax1.plot(r[view_mask], np.zeros(view_mask.sum()), lw=1.2, color=C_PROFILE)
    vline = ax1.axvline(0, color=C_CHEST, ls="--", lw=1.4)
    ax1.set_xlabel("range (m)"); ax1.set_ylabel("magnitude")
    ax1.set_xlim(0, RANGE_VIEW_M)
    ax1.set_title("range profile - locking chest bin...")

    (stick,) = ax2.plot([0, 0], [0, 0], "-", lw=1.8, color=C_PHASOR)
    (tip,) = ax2.plot([0], [0], "o", ms=7, color=C_PHASOR)
    trail = ax2.scatter([], [], s=9, c=C_TRAIL, alpha=0.45)
    ax2.axhline(0, color="k", lw=0.5); ax2.axvline(0, color="k", lw=0.5)
    ax2.set_aspect("equal"); ax2.set_xlabel("Re"); ax2.set_ylabel("Im")
    ax2.set_title("chest bin phasor")

    (pline,) = ax3.plot([], [], lw=1.1, color=C_RAW)
    ax3.set_xlabel("time (s)"); ax3.set_ylabel("displacement (mm)")
    ax3.set_title("chest wall displacement")
    ax3.grid(alpha=0.3)

    (fline,) = ax4.plot([], [], lw=1.2, color=C_HEART)
    peak_line = ax4.axvline(0, color=C_HEART, ls="--", lw=1.2)
    peak_txt = ax4.text(0, 1.0, "", fontsize=10, ha="center", color=C_HEART)
    ax4.axvspan(HEART_LO_HZ, HEART_HI_HZ, color=C_HEART, alpha=0.07)
    ax4.set_xlim(0, SPEC_VIEW_HZ); ax4.set_ylim(0, 1.08)
    ax4.set_xlabel("frequency (Hz)"); ax4.set_ylabel("normalized magnitude")
    ax4.set_title("FFT")
    ax4.grid(alpha=0.3)

    fig.tight_layout(pad=1.5, h_pad=2.8)

    def relock():
        st["chest"] = None; st["wa"] = st["wb"] = None
        st["acc"][:] = 0.0; st["nacc"] = 0
        st["prev_z"] = None; st["phase"] = 0.0; st["n"] = 0; st["clip"] = 0
        buf_n.clear(); buf_z.clear(); buf_d.clear()

    def on_key(ev):
        if ev.key == "r":
            relock()
            ax1.set_title("range profile - re-locking chest bin...")
    fig.canvas.mpl_connect("key_press_event", on_key)

    def process(raw):
        st["last_raw"] = raw
        if st["chest"] is None:                       # locking: needs the profile
            st["acc"] += np.abs(full_profile(raw, rwin))
            st["nacc"] += 1
            if st["nacc"] >= N_LOCK:
                prof = st["acc"] / st["nacc"]
                sel = np.where((r >= CHEST_MIN_M) & (r <= CHEST_MAX_M))[0]
                st["chest"] = int(sel[np.argmax(prof[sel])])
                st["wa"], st["wb"] = make_weights(st["chest"], rwin)
                print(f"chest bin locked: {st['chest']} = {r[st['chest']]:.2f} m "
                      f"({st['nacc']} frames)")
            return
        z = chest_value(frame_to_float(raw), st["wa"], st["wb"])   # the fast path
        if st["prev_z"] is not None:                  # incremental DACM, O(1)
            i0, q0 = st["prev_z"].real, st["prev_z"].imag
            p0 = i0 * i0 + q0 * q0
            if p0 > 1e-9:                             # skip a near-null vector
                di, dq = z.real - i0, z.imag - q0
                inc = (i0 * dq - q0 * di) / p0
                if abs(inc) > MAX_STEP_RAD:           # noise spike, not motion
                    inc = 0.0
                    st["clip"] += 1
                st["phase"] += inc
        st["prev_z"] = z
        buf_n.append(st["n"]); buf_z.append(z); buf_d.append(st["phase"] * MM_PER_RAD)
        st["n"] += 1

    def redraw():
        if st["last_raw"] is not None:
            prof = np.abs(full_profile(st["last_raw"], rwin))
            ln.set_ydata(prof[view_mask])
            ax1.set_ylim(0, 1.05 * prof[view_mask].max() + 1e-9)
        if st["chest"] is None:
            ax1.set_title(f"range profile - locking ({st['nacc']}/{N_LOCK} frames)")
            return
        vline.set_xdata([r[st["chest"]], r[st["chest"]]])
        ax1.set_title(f"range profile  |  chest {r[st['chest']]:.2f} m")

        if len(buf_d) < 16:
            return
        z = np.asarray(buf_z); d = np.asarray(buf_d); ts = np.asarray(buf_n) / fs

        cur = z[-1]
        stick.set_data([0, cur.real], [0, cur.imag])
        tip.set_data([cur.real], [cur.imag])
        tr = z[-TRAIL_LEN:]
        trail.set_offsets(np.c_[tr.real, tr.imag])
        lim = 1.2 * max(np.abs(tr).max(), 1.0)
        ax2.set_xlim(-lim, lim); ax2.set_ylim(-lim, lim)

        # panel 3: RAW DACM, breathing included (only the mean is removed)
        d_show = d - d.mean()
        step = max(1, len(d_show) // PLOT_MAX_PTS)     # fewer artists, same shape
        pline.set_data(ts[::step], d_show[::step])
        lo, hi = float(d_show[::step].min()), float(d_show[::step].max())
        pad = 0.05 * (hi - lo) + 1e-3
        ax3.set_ylim(lo - pad, hi + pad)
        ax3.set_xlim(max(0, ts[-1] - PHASE_WINDOW_S), ts[-1] + 0.1)

        # panel 4: heart = highpass first, then FFT (same as the notebooks)
        nh = min(len(d), NFFT)
        if nh < 64:
            return
        d_hp = hp_filter(d[-nh:])
        f, X = spectrum(d_hp, fs)
        sel = f <= SPEC_VIEW_HZ
        fline.set_data(f[sel], X[sel] / (X[sel].max() + 1e-12))
        fp, _ = band_peak(f, X, HEART_LO_HZ, HEART_HI_HZ)

        bits = [f"heart FFT {nh/fs:.0f} s (res {fs/nh*60:.1f} bpm)"]
        if np.isfinite(fp):
            peak_line.set_xdata([fp, fp])
            peak_txt.set_position((fp, 1.02)); peak_txt.set_text(f"{fp*60:.0f} bpm")
            bits.append(f"heart {fp*60:.0f} bpm")
        ax4.set_title("  |  ".join(bits), fontsize=10)

    fr = open(bin_path, "rb")
    pending = []

    def update(_frame):
        try:
            if stop_evt.is_set() or not plt.fignum_exists(fig.number):
                plt.close(fig)
                return []
            try:
                while True:
                    pending.append(pub_q.get_nowait())
            except queue.Empty:
                pass
            if len(pending) > MAX_BACKLOG:              # never fall behind the live edge
                st["drop"] += len(pending) - MAX_BACKLOG
                st["n"] += len(pending) - MAX_BACKLOG   # keep the time base honest
                del pending[:-MAX_BACKLOG]
            still, brk = [], False
            for fid, off, got in pending:
                if brk:
                    still.append((fid, off, got)); continue
                if got < CHIRPS_PER_FRAME:
                    continue
                fr.seek(off)
                raw = fr.read(FRAME_BYTES)
                if len(raw) < FRAME_BYTES:
                    still.append((fid, off, got)); brk = True; continue
                process(raw)
            pending[:] = still
            redraw()
        except Exception:
            import traceback
            traceback.print_exc()
        return []

    ani = animation.FuncAnimation(fig, update,
                                  interval=int(REDRAW_PERIOD * 1000),
                                  blit=False, cache_frame_data=False)
    fig._vitals_ani = ani
    try:
        plt.show()
    except Exception:
        pass
    finally:
        fr.close()


def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bin_path = f"_live_{stamp}.bin"          # transport only, deleted on exit

    pub_q = mp.Queue(maxsize=PUB_Q_MAX)
    stop_evt = mp.Event()
    disp = mp.Process(target=display_proc, args=(bin_path, pub_q, stop_evt),
                      daemon=True)
    disp.start()

    try:
        capture(bin_path, pub_q)
    finally:
        stop_evt.set()
        disp.join(timeout=2.0)
        if disp.is_alive():
            disp.terminate()
        for _ in range(10):
            try:
                os.remove(bin_path)
                print(f"removed {bin_path}")
                break
            except FileNotFoundError:
                break
            except PermissionError:
                time.sleep(0.2)


if __name__ == "__main__":
    mp.freeze_support()
    main()