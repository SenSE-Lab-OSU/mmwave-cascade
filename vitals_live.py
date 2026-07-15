"""
vitals_live.py - live chest-phase display, built on capture_range_doppler.py

The capture half is reused UNCHANGED by importing capture() and its constants.
Only the display is replaced.

Per frame we coherently sum all 64 loops x 48 channels (6 TX x 8 RX) after a
range FFT, giving one complex value Z[bin] per range bin. The chest bin's slow-
time series (one Z per frame, 20 fps) is what carries respiration/heartbeat.

Panels:
  1  range profile |Z| (raw), chest bin + 2 neighbors marked
  2  phasor: the raw complex chest vector Z (+ neighbors) with a short trail,
     autoscaled. If the phase oscillates you see the vector's tip sweep an arc.
  3  demodulated phase (mean-removed). DEMOD selects which:
       "raw"/"A"/"B"/"C" -> that one method, shown for chest AND its 2 neighbors
       "all"             -> the three methods A,B,C, chest bin only
  4  sliding-window FFT of the panel-3 series; the peak inside the heart band is
     marked with a vertical line and labelled (Hz / bpm). The FFT is taken over
     the last FFT_WINDOW_S seconds only (a moving window), recomputed each
     redraw -> resolution is fixed at 1/FFT_WINDOW_S.

Demodulations (all from the chest bin's complex slow-time Z):
  raw                   : unwrap(angle(Z))                  [no subtraction]
  A subtract-then-angle : unwrap(angle(Z - movavg(Z)))     [winds on a quiet bin]
  B angle-then-subtract : unwrap(angle(Z)) - movavg(unwrap(angle(Z)))
  C DACM                : differentiate-cross-multiply + accumulate (Wang 2014);
                          no arctan/unwrap, can't wind. DEFAULT.

Set DEMOD below to choose which (no live switching). To re-pick the bin change
CHEST_RANGE_M (or set it None to auto-detect the strongest mover in the window).
"""

import os
import queue
import time
import datetime
import multiprocessing as mp
from collections import deque

import numpy as np

from capture_range_doppler import (
    capture, write_csv, verify_bin, summarize,
    SAMPLES_PER_CHIRP, NUM_RX, NUM_TX, CHIRPS_PER_FRAME,
    BLOCK_BYTES, DATA_PER_CHIRP,
    C_LIGHT, ADC_RATE_KSPS, SLOPE_MHZ_US, START_FREQ_GHZ, EXPECTED_DT_MS,
)

# ============================================================
# TUNABLES
# ============================================================
CHEST_RANGE_M = 2            # fixed chest range (m); None = auto-detect peak
CHEST_MIN_M   = 0.3          # auto-detect search window
CHEST_MAX_M   = 2.5
MA_FRAMES     = 20           # moving-average length (frames) for A / phasor
DEMOD         = "C"          # default demod: "raw", "A", "B", "C", or "all"
PHASOR_MODE   = "raw"        # panel 2 vector: "raw" (Z) or "ma" (Z - movavg(Z))
PHASE_WINDOW_S = 60.0        # scrolling window shown in panel 3 (s)
FFT_WINDOW_S  = 30.0         # window the FFT is taken over (s); res = 1/this
HEART_LO_HZ   = 0.8          # heart band for the peak readout (Hz)
HEART_HI_HZ   = 2.0
SPEC_VIEW_HZ  = 2.5          # panel 4 x-axis limit (Hz)
TRAIL_LEN     = 40           # phasor trail length (frames)
REDRAW_PERIOD = 0.5          # seconds between redraws (processing stays per-frame)
PUB_Q_MAX     = 64           # frame-handoff queue depth
LAYOUT_B      = False        # int16 interleave: flip if range profile looks wrong
RANGE_VIEW_M  = 4.0          # panel 1 x-axis limit (m)

FRAME_PERIOD_S = EXPECTED_DT_MS / 1000.0
FRAME_BYTES    = CHIRPS_PER_FRAME * BLOCK_BYTES
LAMBDA         = C_LIGHT / (START_FREQ_GHZ * 1e9)   # for phase->mm note only


def range_axis():
    fs = ADC_RATE_KSPS * 1e3
    S  = SLOPE_MHZ_US * 1e12
    dR = C_LIGHT * fs / (2.0 * S * SAMPLES_PER_CHIRP)
    return np.arange(SAMPLES_PER_CHIRP) * dR


def frame_complex_profile(raw, rwin):
    """Full frame -> coherent complex range profile Z (SAMPLES_PER_CHIRP,)."""
    nch = CHIRPS_PER_FRAME
    m = np.frombuffer(raw, dtype=np.uint8).reshape(nch, BLOCK_BYTES)
    data = np.ascontiguousarray(m[:, :DATA_PER_CHIRP])   # strip 16-byte headers
    iq = data.view(np.int16).astype(np.float32)          # (nch, 4096)
    c = iq[:, 0::2] + 1j * iq[:, 1::2]                    # (nch, 2048)
    if not LAYOUT_B:
        c = c.reshape(nch, NUM_RX, SAMPLES_PER_CHIRP)                    # LAYOUT A
    else:
        c = c.reshape(nch, SAMPLES_PER_CHIRP, NUM_RX).transpose(0, 2, 1)  # LAYOUT B
    rng = np.fft.fft(c * rwin[None, None, :], axis=2)    # (nch, rx, bins)
    return rng.sum(axis=(0, 1))                          # coherent sum -> (bins,)


def detect_bin(prof_mag, r):
    if CHEST_RANGE_M is not None:
        return int(np.argmin(np.abs(r - CHEST_RANGE_M)))
    idx = np.where((r >= CHEST_MIN_M) & (r <= CHEST_MAX_M))[0]
    return int(idx[np.argmax(prof_mag[idx])])


# ---- demodulation ----
def movavg_trailing(x, w):
    """Trailing moving average along a 1-D array (expanding during warmup)."""
    n = len(x)
    if w <= 1 or n == 0:
        return np.asarray(x, float)
    c = np.cumsum(x)
    out = np.empty(n)
    k = min(w, n)
    out[:k] = c[:k] / np.arange(1, k + 1)
    if n > w:
        out[w:] = (c[w:] - c[:-w]) / w
    return out


def dacm(z):
    """Extended DACM phase (Wang 2014, eq. 9): differentiate-cross-multiply then
    accumulate. phi[n] = sum ( I dQ - dI Q ) / (I^2 + Q^2). No arctan, no unwrap,
    so it can't wind on a near-zero vector; static clutter drops out under diff."""
    n = len(z)
    if n < 2:
        return np.zeros(n)
    I = z.real.astype(float)
    Q = z.imag.astype(float)
    dI = np.diff(I)
    dQ = np.diff(Q)
    omega = (I[1:] * dQ - dI * Q[1:]) / (I[1:] ** 2 + Q[1:] ** 2 + 1e-12)
    return np.concatenate([[0.0], np.cumsum(omega)])


def demod_one(zraw, zres, which):
    """One demodulation of one bin. zraw = raw complex; zres = MA residual."""
    if which == "raw":
        return np.unwrap(np.angle(zraw))                # raw angle, no subtraction
    if which == "A":
        return np.unwrap(np.angle(zres))
    if which == "B":
        p = np.unwrap(np.angle(zraw))
        return p - movavg_trailing(p, MA_FRAMES)
    return dacm(zraw)                                    # "C"


def spectrum(x, fs):
    """Magnitude spectrum of x: mean-remove, Hann window, real FFT."""
    x = (np.asarray(x, float) - np.mean(x)) * np.hanning(len(x))
    X = np.abs(np.fft.rfft(x))
    f = np.fft.rfftfreq(len(x), d=1.0 / fs)
    return f, X


def band_peak(f, X, lo, hi):
    """Parabolically-refined peak of X within [lo, hi]. Returns (freq, height)."""
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
    NW = int(PHASE_WINDOW_S / FRAME_PERIOD_S) + 1
    NFFT = max(16, int(FFT_WINDOW_S * fs))

    st = {"chest": None, "t0": None, "Z": None, "res": None,
          "demod": DEMOD, "demod_prev": None}
    ma = deque(maxlen=MA_FRAMES)
    buf_t = deque(maxlen=NW)
    buf_raw = deque(maxlen=NW)             # each: 3 complex (bin-1, bin, bin+1)
    buf_res = deque(maxlen=NW)

    def bins3():
        b = st["chest"]
        return [max(0, b - 1), b, min(SAMPLES_PER_CHIRP - 1, b + 1)]

    def series_for(zr, zsr):
        d = st["demod"]
        if d == "all":
            A = demod_one(zr[:, 1], zsr[:, 1], "A")
            B = demod_one(zr[:, 1], zsr[:, 1], "B")
            C = demod_one(zr[:, 1], zsr[:, 1], "C")
            return [A, B, C], ["A sub-then-ang", "B ang-then-sub", "C DACM"], \
                   ["C3", "C0", "C2"], 2
        s = [demod_one(zr[:, j], zsr[:, j], d) for j in range(3)]
        return s, ["chest-1", "chest", "chest+1"], ["C0", "C1", "C2"], 1

    # ---- figure: wide 2x2 ----
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    ax1, ax2, ax3, ax4 = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]
    bcol = ["C0", "C1", "C2"]
    blbl = ["chest-1", "chest", "chest+1"]

    (ln,) = ax1.plot(r, np.zeros_like(r))
    vlines = [ax1.axvline(0, color=bcol[j], ls="--", lw=1) for j in range(3)]
    ax1.set_xlabel("range (m)"); ax1.set_ylabel("|Z| (a.u.)")
    ax1.set_xlim(0, RANGE_VIEW_M)

    sticks = [ax2.plot([0, 0], [0, 0], "-", lw=1.5, color=bcol[j], label=blbl[j])[0]
              for j in range(3)]
    tips = [ax2.plot([0], [0], "o", color=bcol[j])[0] for j in range(3)]
    trail = ax2.scatter([], [], s=8, c="gray", alpha=0.35)
    ax2.axhline(0, color="k", lw=0.5); ax2.axvline(0, color="k", lw=0.5)
    ax2.set_aspect("equal"); ax2.set_xlabel("Re"); ax2.set_ylabel("Im")
    ax2.set_title(f"phasor ({'Z - movavg' if PHASOR_MODE == 'ma' else 'raw Z'})")
    ax2.legend(loc="upper right", fontsize=8)

    plines = [ax3.plot([], [], lw=1)[0] for _ in range(3)]
    ax3.set_xlabel("time (s)"); ax3.set_ylabel("phase (rad, mean-removed)")

    pflines = [ax4.plot([], [], lw=1)[0] for _ in range(3)]
    peak_line = ax4.axvline(0, color="k", ls="--", lw=1)
    peak_txt = ax4.text(0, 1.0, "", fontsize=9, ha="center")
    ax4.axvspan(HEART_LO_HZ, HEART_HI_HZ, color="C3", alpha=0.07)
    ax4.set_xlim(0, SPEC_VIEW_HZ); ax4.set_ylim(0, 1.08)
    ax4.set_xlabel("frequency (Hz)"); ax4.set_ylabel("normalized magnitude")

    fig.tight_layout(pad=1.5, h_pad=2.8)

    def process(raw, fid):
        Z = frame_complex_profile(raw, rwin)
        ma.append(Z)
        res = Z - (np.mean(ma, axis=0) if len(ma) else 0.0)
        st["Z"], st["res"] = Z, res
        if st["chest"] is None:
            st["chest"] = detect_bin(np.abs(res), r)
            print(f"locked chest bin {st['chest']} (range {r[st['chest']]:.2f} m)")
        if st["t0"] is None:
            st["t0"] = fid
        b = bins3()
        buf_t.append((fid - st["t0"]) * FRAME_PERIOD_S)
        buf_raw.append(Z[b])
        buf_res.append(res[b])

    def redraw():
        # sets every artist AND rescales axes on every call (no throttling)
        prof = np.abs(st["Z"])
        ln.set_ydata(prof)
        ax1.set_ylim(0, 1.05 * prof[view_mask].max() + 1e-9)
        for j, bb in enumerate(bins3()):
            vlines[j].set_xdata([r[bb], r[bb]])
        ax1.set_title(f"range profile (raw)  |  chest {r[st['chest']]:.2f} m")

        zr = np.array(buf_raw); zsr = np.array(buf_res); ts = np.array(buf_t)
        if len(ts) < 2:
            return

        zph = zsr if PHASOR_MODE == "ma" else zr    # panel-2 vector source
        cur = zph[-1]
        for j in range(3):
            sticks[j].set_data([0, cur[j].real], [0, cur[j].imag])
            tips[j].set_data([cur[j].real], [cur[j].imag])
        tr = zph[-TRAIL_LEN:, 1]
        trail.set_offsets(np.c_[tr.real, tr.imag])
        lim = 1.2 * max(np.abs(cur).max(), np.abs(tr).max(), 1.0)
        ax2.set_xlim(-lim, lim); ax2.set_ylim(-lim, lim)

        series, labels, colors, prim = series_for(zr, zsr)
        if st["demod"] != st["demod_prev"]:
            st["demod_prev"] = st["demod"]
            for j in range(3):
                plines[j].set_color(colors[j]); plines[j].set_label(labels[j])
                pflines[j].set_color(colors[j]); pflines[j].set_label(labels[j])
            ax3.legend(loc="upper left", fontsize=8)
            ax4.legend(loc="upper right", fontsize=8)
            tag = ("A/B/C (chest)" if st["demod"] == "all"
                   else f"demod {st['demod']} (chest & neighbors)")
            ax3.set_title(f"phase - {tag}")

        ymin = ymax = 0.0
        for j in range(3):
            d = series[j] - series[j].mean()
            plines[j].set_data(ts, d)
            ymin = min(ymin, d.min()); ymax = max(ymax, d.max())
        pad = 0.05 * (ymax - ymin) + 1e-6
        ax3.set_ylim(ymin - pad, ymax + pad)
        ax3.set_xlim(max(0, ts[-1] - PHASE_WINDOW_S), ts[-1] + 0.1)

        nfft = min(len(ts), NFFT)
        specs = [spectrum(series[j][-nfft:], fs) for j in range(3)]
        for j, (f, X) in enumerate(specs):
            Xn = X / (X.max() + 1e-12)
            sel = f <= SPEC_VIEW_HZ
            pflines[j].set_data(f[sel], Xn[sel])
        fp, _ = band_peak(*specs[prim], HEART_LO_HZ, HEART_HI_HZ)
        if np.isfinite(fp):
            peak_line.set_xdata([fp, fp])
            peak_txt.set_position((fp, 1.0))
            peak_txt.set_text(f"{fp:.2f} Hz / {fp*60:.0f} bpm")
            ax4.set_title(f"FFT ({nfft/fs:.0f}s, res {fs/nfft*60:.1f} bpm)"
                          f"  |  heart {fp*60:.0f} bpm")
        else:
            ax4.set_title(f"FFT ({nfft/fs:.0f}s)")

    fr = open(bin_path, "rb")
    pending = []

    def update(_frame):
        # Driven by FuncAnimation's steady timer on the GUI thread: window stays
        # responsive, and the draw happens right after this returns each tick.
        try:
            if stop_evt.is_set() or not plt.fignum_exists(fig.number):
                plt.close(fig)
                return []
            try:
                while True:
                    pending.append(pub_q.get_nowait())
            except queue.Empty:
                pass
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
                process(raw, fid)
            pending[:] = still
            if st["chest"] is not None:
                redraw()
        except Exception:
            import traceback
            traceback.print_exc()
        return []

    ani = animation.FuncAnimation(fig, update,
                                  interval=int(REDRAW_PERIOD * 1000),
                                  blit=False, cache_frame_data=False)
    fig._vitals_ani = ani     # keep a reference so it isn't garbage-collected
    try:
        plt.show()            # blocks; GUI event loop owns the thread
    except Exception:
        pass
    finally:
        fr.close()


def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bin_path = f"capture_{stamp}.bin"
    csv_path = f"capture_{stamp}.csv"

    pub_q = mp.Queue(maxsize=PUB_Q_MAX)
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
