"""
vitals_live.py - live chest-phase display, built on capture_range_doppler.py

The capture half is reused UNCHANGED by importing capture() and its constants.
Only the display is replaced.

Per frame we coherently sum all 64 loops x 48 channels (6 TX x 8 RX) after a
range FFT, giving one complex value Z[bin] per range bin:
    |Z|      -> range profile (panel 1)
    angle(Z) -> phase          (panels 2, 3, 4)
This is an unsteered (boresight) beam. Fine if the subject sits near boresight;
an off-axis chest will partially cancel across the aperture.

The phase series is sampled at the full frame rate (20 fps), not the redraw
rate, so it is usable for respiration/heartbeat. Redraw is throttled separately.

Panels:
  1  range profile |Z|, chest bin + 2 neighbors marked
  2  per-frame phase vector (phasor) for the 3 bins, with a short trail
  3  unwrapped phase vs time for the 3 bins
  4  the same phase, bandpassed BAND_LO_HZ..BAND_HI_HZ (drift/DC and out-of-band
     noise removed), recomputed over the visible window each redraw

Modes (toggle live):
  m  moving-average subtraction: residual = Z - mean(last MA_FRAMES frames).
     Removes static clutter, leaves movers; chest auto-detect is far better
     here. NOTE: an MA_FRAMES=20 (1 s) subtraction high-passes ~0.4 Hz, so it
     suppresses slow respiration and keeps heartbeat. Raise MA_FRAMES to keep
     respiration.
  r  raw (no subtraction)
  d  re-detect + lock the chest bin from the current mode's profile
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
CHEST_RANGE_M = 2          # fixed chest range (m); None = auto-detect peak
CHEST_MIN_M   = 0.3           # auto-detect search window
CHEST_MAX_M   = 2.5
MA_FRAMES     = 20            # moving-average length (frames) for subtraction
START_MODE    = "raw"        # "raw" or "ma"
PHASE_WINDOW_S = 60.0         # scrolling window for the phase time series
TRAIL_LEN     = 40           # phasor trail length (frames)
REDRAW_PERIOD = 0.3          # seconds between redraws (processing is per-frame)
PUB_Q_MAX     = 64           # frame-handoff queue depth
LAYOUT_B      = False        # int16 interleave: flip if range profile looks wrong

# ---- bandpass on the displayed unwrapped phase (panel 4) ----
BAND_LO_HZ    = 0.4          # passband edges (Hz)
BAND_HI_HZ    = 3.0
BAND_TRANS_HZ = 0.1          # cosine roll-off width on each side of the band

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


def _band_mask(f, lo, hi, trans):
    """Raised-cosine bandpass mask over frequency bins f."""
    m = np.zeros_like(f)
    m[(f >= lo) & (f <= hi)] = 1.0
    if trans > 0:
        rise = (f >= lo - trans) & (f < lo)
        m[rise] = 0.5 * (1 - np.cos(np.pi * (f[rise] - (lo - trans)) / trans))
        fall = (f > hi) & (f <= hi + trans)
        m[fall] = 0.5 * (1 + np.cos(np.pi * (f[fall] - hi) / trans))
    return m


def bandpass_phase(x, fs, lo, hi, trans):
    """Frequency-domain bandpass of one unwrapped-phase track. Linear-detrended
    first (kills DC + drift below the band); NO taper window, so the newest
    sample keeps its true value for the live view. Zero-phase; the two edges
    ring a little, so treat the far right (newest ~1 s) as least settled."""
    n = len(x)
    if n < 8:
        return np.zeros_like(x)
    tt = np.arange(n)
    a, b = np.polyfit(tt, x, 1)
    X = np.fft.rfft(x - (a * tt + b))
    f = np.fft.rfftfreq(n, d=1.0 / fs)
    return np.fft.irfft(X * _band_mask(f, lo, hi, trans), n)


# ============================================================
# Display process
# ============================================================
def display_proc(bin_path, pub_q, stop_evt):
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"display: matplotlib unavailable ({e}); capturing without a view.")
        return

    while not os.path.exists(bin_path):
        if stop_evt.is_set():
            return
        time.sleep(0.1)

    r = range_axis()
    rwin = np.hanning(SAMPLES_PER_CHIRP).astype(np.float32)
    fs = 1.0 / FRAME_PERIOD_S
    NW = int(PHASE_WINDOW_S / FRAME_PERIOD_S) + 1

    st = {
        "mode": START_MODE,
        "chest": None,          # locked chest bin
        "redetect": False,
        "t0": None,
        "Z": None,              # last complex profile
        "res": None,            # last residual profile
    }
    ma = deque(maxlen=MA_FRAMES)            # complex profiles for the moving avg
    buf_t = deque(maxlen=NW)
    buf_raw = deque(maxlen=NW)             # each: 3 complex (bin-1, bin, bin+1)
    buf_res = deque(maxlen=NW)

    def bins3():
        b = st["chest"]
        return [max(0, b - 1), b, min(SAMPLES_PER_CHIRP - 1, b + 1)]

    def clear_series():
        buf_t.clear(); buf_raw.clear(); buf_res.clear()

    # ---- figure ----
    plt.ion()
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(7, 12))
    col = ["C0", "C1", "C2"]   # bin-1, chest, bin+1
    lbl = ["chest-1", "chest", "chest+1"]

    (ln,) = ax1.plot(r, np.zeros_like(r))
    vlines = [ax1.axvline(0, color=col[j], ls="--", lw=1) for j in range(3)]
    ax1.set_xlabel("range (m)"); ax1.set_ylabel("|Z| (a.u.)")

    sticks = [ax2.plot([0, 0], [0, 0], "-", lw=1.5, color=col[j], label=lbl[j])[0]
              for j in range(3)]
    tips = [ax2.plot([0], [0], "o", color=col[j])[0] for j in range(3)]
    trail = ax2.scatter([], [], s=8, c="gray", alpha=0.35)
    ax2.axhline(0, color="k", lw=0.5); ax2.axvline(0, color="k", lw=0.5)
    ax2.set_aspect("equal"); ax2.set_xlabel("Re"); ax2.set_ylabel("Im")
    ax2.set_title("per-frame phase vector"); ax2.legend(loc="upper right", fontsize=8)

    plines = [ax3.plot([], [], color=col[j], label=lbl[j])[0] for j in range(3)]
    ax3.set_xlabel("time (s)"); ax3.set_ylabel("unwrapped phase (rad)")
    ax3.set_title("unwrapped phase"); ax3.legend(loc="upper left", fontsize=8)

    pflines = [ax4.plot([], [], color=col[j], label=lbl[j])[0] for j in range(3)]
    ax4.axhline(0, color="k", lw=0.5)
    ax4.set_xlabel("time (s)"); ax4.set_ylabel("bandpassed phase (rad)")
    ax4.set_title(f"bandpassed {BAND_LO_HZ}-{BAND_HI_HZ} Hz")
    ax4.legend(loc="upper left", fontsize=8)

    def on_key(evt):
        if evt.key == "m":
            st["mode"] = "ma"
        elif evt.key == "r":
            st["mode"] = "raw"
        elif evt.key == "d":
            st["redetect"] = True
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.tight_layout()

    def process(raw, fid):
        Z = frame_complex_profile(raw, rwin)
        ma.append(Z)
        res = Z - (np.mean(ma, axis=0) if len(ma) else 0.0)
        st["Z"], st["res"] = Z, res

        prof = np.abs(res) if st["mode"] == "ma" else np.abs(Z)
        if st["chest"] is None or st["redetect"]:
            st["chest"] = detect_bin(prof, r)
            st["redetect"] = False
            clear_series()
            print(f"locked chest bin {st['chest']} "
                  f"(range {r[st['chest']]:.2f} m, mode {st['mode']})")

        if st["t0"] is None:
            st["t0"] = fid
        b = bins3()
        buf_t.append((fid - st["t0"]) * FRAME_PERIOD_S)
        buf_raw.append(Z[b])
        buf_res.append(res[b])

    # ---- main loop ----
    fr = open(bin_path, "rb")
    pending = []
    next_draw = 0.0
    try:
        while not stop_evt.is_set():
            if not plt.fignum_exists(fig.number):
                break
            try:
                while True:
                    pending.append(pub_q.get_nowait())
            except queue.Empty:
                pass

            still, brk = [], False
            for fid, off, got in pending:
                if brk:
                    still.append((fid, off, got)); continue
                if got < CHIRPS_PER_FRAME:            # incomplete frame -> drop
                    continue
                fr.seek(off)
                raw = fr.read(FRAME_BYTES)
                if len(raw) < FRAME_BYTES:            # not flushed yet -> retry
                    still.append((fid, off, got)); brk = True; continue
                process(raw, fid)
            pending = still

            now = time.perf_counter()
            if now >= next_draw and st["chest"] is not None:
                next_draw = now + REDRAW_PERIOD
                mode = st["mode"]
                prof = np.abs(st["res"]) if mode == "ma" else np.abs(st["Z"])
                ln.set_ydata(prof)
                ax1.relim(); ax1.autoscale_view(scalex=False)
                for j, bb in enumerate(bins3()):
                    vlines[j].set_xdata([r[bb], r[bb]])
                ax1.set_title(f"range profile  |  chest {r[st['chest']]:.2f} m  "
                              f"|  mode: {mode}")

                zs = np.array(buf_res if mode == "ma" else buf_raw)  # (N,3)
                ts = np.array(buf_t)
                if len(zs):
                    cur = zs[-1]
                    lim = 1.2 * max(np.abs(cur).max(), 1.0)
                    for j in range(3):
                        sticks[j].set_data([0, cur[j].real], [0, cur[j].imag])
                        tips[j].set_data([cur[j].real], [cur[j].imag])
                    tr = zs[-TRAIL_LEN:, 1]
                    trail.set_offsets(np.c_[tr.real, tr.imag])
                    lim = max(lim, 1.2 * np.abs(tr).max())
                    ax2.set_xlim(-lim, lim); ax2.set_ylim(-lim, lim)

                    ph = np.unwrap(np.angle(zs), axis=0)   # (N,3)
                    for j in range(3):
                        plines[j].set_data(ts, ph[:, j])
                    ax3.relim(); ax3.autoscale_view()

                    for j in range(3):                     # bandpassed phase
                        bp = bandpass_phase(ph[:, j], fs,
                                            BAND_LO_HZ, BAND_HI_HZ, BAND_TRANS_HZ)
                        pflines[j].set_data(ts, bp)
                    ax4.relim(); ax4.autoscale_view()

                    if len(ts):
                        x0, x1 = max(0, ts[-1] - PHASE_WINDOW_S), ts[-1] + 0.1
                        ax3.set_xlim(x0, x1)
                        ax4.set_xlim(x0, x1)

            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        fr.close()
        try:
            plt.ioff(); plt.close(fig)
        except Exception:
            pass


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