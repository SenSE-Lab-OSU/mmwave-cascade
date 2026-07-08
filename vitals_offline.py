"""
vitals_offline.py - same chest-phase analysis as vitals_live.py, but on an
already-saved .bin file. No capture, no live loop: you pick a frame range and a
range bin, it processes that segment and plots the whole thing at once.

Reuses the DSP from vitals_live.py (range_axis, frame_complex_profile).

Set the three knobs at the top:
  BIN_PATH     which file to read        (or pass as the first CLI argument)
  FRAME_START  first frame to analyze    } 0-based, sequential from file start,
  FRAME_END    last frame (exclusive)    } same numbering as bgsub_3d
  RANGE_M      chest range in meters     (None = auto-detect strongest in window)
  MODE         "raw" or "ma"             (ma = subtract 20-frame moving average)

Panels (identical meaning to the live view, but non-scrolling over the whole
selected span):
  1  range profile |Z| averaged over the segment, chest bin +/- 1 marked
  2  FFT magnitude spectrum of the chest phase, before vs after the bandpass,
     with the passband shaded and the detected peak (= rate) marked
  3  the detrended chest phase and the band-limited signal extracted from it
     (the oscillation that survives the filter)

Set BAND_LO_HZ/BAND_HI_HZ to breathing or heartbeat, and pick MODE to match
(raw for breathing, ma for heartbeat).
"""

import sys
import struct

import numpy as np

from vitals_live import range_axis, frame_complex_profile, FRAME_PERIOD_S, LAMBDA
from capture_range_doppler import (
    SAMPLES_PER_CHIRP, CHIRPS_PER_FRAME, BLOCK_BYTES, DATA_PER_CHIRP,
    HEADER_BYTES, MAGIC_LE,
)

# ============================================================
# TUNABLES
# ============================================================
BIN_PATH    = "capture.bin"   # overridden by argv[1] if given
FRAME_START = 900               # first frame (inclusive), 0-based
FRAME_END   = 1650            # last frame (exclusive); None = end of file
RANGE_M     = 2            # fixed chest range (m); None = auto-detect
CHEST_MIN_M = 0.3             # auto-detect search window
CHEST_MAX_M = 2.5
MA_FRAMES   = 20              # moving-average length for MODE="ma"
MODE        = "ma"            # "raw" or "ma"

# ---- bandpass + FFT rate detection (on the chest bin's unwrapped phase) ----
# Passband = what you keep. Set it to what you're measuring, and pick MODE:
#   respiration  0.1-0.6 Hz (6-36 br/min)  -> use MODE="raw"
#   heartbeat    0.8-2.0 Hz (48-120 bpm)   -> use MODE="ma"
BAND_LO_HZ   = 0.8           # passband lower edge (Hz)
BAND_HI_HZ   = 2.0           # passband upper edge (Hz)
BAND_TRANS_HZ = 0.1          # cosine roll-off width on each side of the band

FRAME_BYTES = CHIRPS_PER_FRAME * BLOCK_BYTES


def index_frame_offsets(path):
    """Return (frame_offsets, n_frames, n_chirps). Frames are fixed 384-chirp
    blocks in file order; a stride check warns if chirps were dropped."""
    with open(path, "rb") as fp:
        data = fp.read()
    offs = []
    idx = data.find(MAGIC_LE)
    while idx != -1:
        if idx + HEADER_BYTES > len(data):
            break
        if idx >= DATA_PER_CHIRP:                 # header sits after the 8192 data
            offs.append(idx - DATA_PER_CHIRP)     # -> start of this chirp's data
        idx = data.find(MAGIC_LE, idx + HEADER_BYTES)
    offs = np.asarray(offs, dtype=np.int64)
    if offs.size >= 2:
        bad = np.count_nonzero(np.diff(offs) != BLOCK_BYTES)
        if bad:
            print(f"WARN: {bad} chirp gaps != {BLOCK_BYTES} B "
                  "(dropped/spurious chirps); frame alignment may be off.")
    nfr = offs.size // CHIRPS_PER_FRAME
    frame_off = offs[:nfr * CHIRPS_PER_FRAME:CHIRPS_PER_FRAME]
    return frame_off, nfr, offs.size


def load_series(path, frame_off, f0, f1):
    """Read frames [f0, f1) -> complex Z series (n, SAMPLES_PER_CHIRP)."""
    rwin = np.hanning(SAMPLES_PER_CHIRP).astype(np.float32)
    out = np.empty((f1 - f0, SAMPLES_PER_CHIRP), np.complex128)
    with open(path, "rb") as fp:
        for i, f in enumerate(range(f0, f1)):
            fp.seek(int(frame_off[f]))
            raw = fp.read(FRAME_BYTES)
            if len(raw) < FRAME_BYTES:
                out = out[:i]
                print(f"stopped at frame {f}: only {len(raw)} B available.")
                break
            out[i] = frame_complex_profile(raw, rwin)
    return out


def moving_avg_trailing(x, w):
    """Trailing moving average along axis 0 (expanding during warmup), to match
    the live view's deque semantics."""
    if w <= 1:
        return x
    c = np.cumsum(x, axis=0)
    out = np.empty_like(x)
    k = min(w, x.shape[0])
    out[:k] = c[:k] / np.arange(1, k + 1)[:, None]
    if x.shape[0] > w:
        out[w:] = (c[w:] - c[:-w]) / w
    return out


def detect_bin(prof_mag, r):
    if RANGE_M is not None:
        return int(np.argmin(np.abs(r - RANGE_M)))
    idx = np.where((r >= CHEST_MIN_M) & (r <= CHEST_MAX_M))[0]
    return int(idx[np.argmax(prof_mag[idx])])


def _detrend(x):
    """Remove a linear trend so the signal oscillates about zero."""
    n = len(x)
    tt = np.arange(n)
    a, b = np.polyfit(tt, x, 1)
    return x - (a * tt + b)


def _band_mask(f, f_lo, f_hi, trans):
    """Raised-cosine bandpass mask: 1 inside [f_lo, f_hi], smoothly -> 0 over a
    'trans'-wide roll-off on each side, 0 elsewhere. Smooth edges avoid the
    time-domain ringing a brick-wall cut would create."""
    m = np.zeros_like(f)
    m[(f >= f_lo) & (f <= f_hi)] = 1.0
    if trans > 0:
        rise = (f >= f_lo - trans) & (f < f_lo)
        m[rise] = 0.5 * (1 - np.cos(np.pi * (f[rise] - (f_lo - trans)) / trans))
        fall = (f > f_hi) & (f <= f_hi + trans)
        m[fall] = 0.5 * (1 + np.cos(np.pi * (f[fall] - f_hi) / trans))
    return m


def bandpass_fft(sig, fs, f_lo, f_hi, trans):
    """Detrend + Hann-window, forward FFT, keep only the passband, and report
    the dominant in-band frequency. Returns everything needed to plot it.

    freqs / spec_raw  : magnitude spectrum of the phase (pre-filter)
    spec_bp           : magnitude spectrum after the bandpass mask
    filtered          : the band-limited signal transformed back to the time
                        domain (this is the extracted oscillation you see)
    rate_hz           : parabolically-refined peak of spec_bp inside the band
    """
    n = len(sig)
    x = _detrend(sig) * np.hanning(n)                 # detrend + window
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, d=1.0 / fs)                # bin freqs: k*fs/n
    mask = _band_mask(f, f_lo, f_hi, trans)
    Xbp = X * mask
    filtered = np.fft.irfft(Xbp, n)                   # back to time domain

    mag = np.abs(Xbp)
    band = np.where((f >= f_lo) & (f <= f_hi))[0]
    if band.size:
        k = band[np.argmax(mag[band])]
        if 0 < k < len(mag) - 1:                      # parabolic sub-bin refine
            a, b, c = mag[k - 1], mag[k], mag[k + 1]
            denom = a - 2 * b + c
            d = 0.5 * (a - c) / denom if denom != 0 else 0.0
        else:
            d = 0.0
        rate_hz = (k + d) * fs / n
    else:
        rate_hz = np.nan
    return {
        "freqs": f,
        "spec_raw": np.abs(X),
        "spec_bp": mag,
        "filtered": filtered,
        "rate_hz": rate_hz,
        "res_hz": fs / n,                             # frequency resolution
    }





def main():
    path = sys.argv[1] if len(sys.argv) > 1 else BIN_PATH
    r = range_axis()

    frame_off, nfr, nch = index_frame_offsets(path)
    print(f"{path}: {nch} chirps -> {nfr} frames "
          f"({nfr * FRAME_PERIOD_S:.1f} s at {1/FRAME_PERIOD_S:.0f} fps)")
    f0 = max(0, FRAME_START)
    f1 = nfr if FRAME_END is None else min(nfr, FRAME_END)
    if f1 <= f0:
        print("empty frame range; check FRAME_START/FRAME_END.")
        return
    print(f"analyzing frames [{f0}, {f1})  ({(f1 - f0) * FRAME_PERIOD_S:.1f} s)")

    Z = load_series(path, frame_off, f0, f1)          # (n, bins) complex
    res = Z - moving_avg_trailing(Z, MA_FRAMES)
    use = res if MODE == "ma" else Z

    prof = np.abs(use).mean(axis=0)
    b = detect_bin(prof, r)
    bins = [max(0, b - 1), b, min(SAMPLES_PER_CHIRP - 1, b + 1)]
    print(f"chest bin {b}  (range {r[b]:.2f} m)  |  mode: {MODE}")

    t = np.arange(Z.shape[0]) * FRAME_PERIOD_S
    zb = use[:, bins]                                   # (n, 3)
    ph = np.unwrap(np.angle(zb), axis=0)

    fs = 1.0 / FRAME_PERIOD_S
    d = bandpass_fft(ph[:, 1], fs, BAND_LO_HZ, BAND_HI_HZ, BAND_TRANS_HZ)  # chest
    rate = d["rate_hz"]
    print(f"passband {BAND_LO_HZ}-{BAND_HI_HZ} Hz  "
          f"({BAND_LO_HZ*60:.0f}-{BAND_HI_HZ*60:.0f} /min)  |  "
          f"resolution {d['res_hz']*60:.1f} /min")
    if np.isfinite(rate):
        print(f"fft rate:  {rate:.3f} Hz  =  {rate*60:.1f} /min")
    else:
        print("fft rate:  nothing in passband")

    import matplotlib.pyplot as plt
    col = ["C0", "C1", "C2"]
    lbl = ["chest-1", "chest", "chest+1"]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 10))

    # panel 1: range profile
    ax1.plot(r, prof)
    for j, bb in enumerate(bins):
        ax1.axvline(r[bb], color=col[j], ls="--", lw=1)
    ax1.set_xlabel("range (m)"); ax1.set_ylabel("|Z| mean (a.u.)")
    ax1.set_title(f"range profile  |  chest {r[b]:.2f} m  |  mode: {MODE}")

    # panel 2: spectrum (raw vs band-passed) with the passband shaded + peak
    f = d["freqs"]
    xr = min(BAND_HI_HZ + 0.5, f[-1])
    m = f <= xr
    ax2.plot(f[m], d["spec_raw"][m], color="0.6", lw=0.8, label="phase spectrum")
    ax2.plot(f[m], d["spec_bp"][m], color="C3", lw=1.4, label="after bandpass")
    ax2.axvspan(BAND_LO_HZ, BAND_HI_HZ, color="C3", alpha=0.08)
    if np.isfinite(rate):
        ax2.axvline(rate, color="k", ls="--", lw=1)
        ax2.annotate(f"{rate:.2f} Hz\n{rate*60:.0f}/min", xy=(rate, 0),
                     xytext=(rate, ax2.get_ylim()[1]*0.7), fontsize=9,
                     ha="center", color="k")
    ax2.set_xlabel("frequency (Hz)"); ax2.set_ylabel("magnitude")
    ax2.set_title("FFT spectrum  (peak in passband = rate)")
    ax2.legend(loc="upper right", fontsize=8)

    # panel 3: detrended phase with the band-limited signal extracted from it
    ax3.plot(t, _detrend(ph[:, 1]), color="0.7", lw=0.8, label="chest phase")
    ax3.plot(t, d["filtered"], color="C3", lw=1.2, label="bandpassed")
    ax3.set_xlabel("time (s)"); ax3.set_ylabel("phase (rad)")
    ttl = "phase over time (detrended) + extracted band"
    if np.isfinite(rate):
        ttl += f"  |  {rate*60:.1f}/min"
    ax3.set_title(ttl)
    ax3.legend(loc="upper left", fontsize=8)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()