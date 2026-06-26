"""
bgsub_3d.py -- ONE self-contained file: background-subtracted 3D point-cloud
movie of a moving target, using the manual steering-vector (Bartlett) angle
search, with per-channel phase calibration computed INLINE from the corner-
reflector stage of the same capture (no separate make_calibration step, no
reference_48ch.npy file needed).

    python bgsub_3d.py capture.bin              # process .bin -> save cache -> movie
    python bgsub_3d.py capture.cache.npz        # REPLAY from cache (fast, no .bin)
    python bgsub_3d.py capture.bin --nocal      # calibration OFF (A/B the effect)
    python bgsub_3d.py capture.bin --nobg       # no background suppression (raw)
    python bgsub_3d.py --selftest               # check the math, no file needed

Processing the .bin detects every target point down to STORE_DB below the frame
peak (and down to a low confidence floor) and writes them to <capture>.cache.npz
(frame, x/y/z, intensity, velocity, dB-below-peak, range, azimuth, elevation,
confidence). Re-running on that .npz rebuilds the movie instantly -- no FFTs --
and DET_DB / MIN_CONF_DB then just re-trim which stored points are shown. So:
capture once, then sweep DET_DB, MIN_CONF_DB, COLOR_BY and the view, re-rendering
in seconds. (Going looser than what was stored needs the .bin again.)

It also writes <capture>.cal.npz (the per-channel phase correction). Point
CAL_FILE at a saved one to REUSE it on a recording that has no reflector -- see
the RF-phase caveat printed when it loads.

There are NO value flags to type. Everything tunable is a global in CONFIG below.
The only command-line switches are: --nocal, --nobg, --selftest.

----------------------------------------------------------------------------
WHAT EACH TUNABLE MEANS  (all live in the CONFIG block right below)
----------------------------------------------------------------------------
Frame ranges (read these off your range_over_frames.png stage map):
  BG_FRAMES     (e0, e1)  empty-scene frames -> averaged into the background
                          template that gets subtracted from every frame.
  TARGET_START  first frame to actually build the cloud from (where the moving
                target stage begins). Runs from here to the end of the file.
  CAL_FRAMES    (f0, f1)  the reflector-at-boresight frames. The cal is built on
                the FIRST half and validated on the SECOND half (non-circular
                check). Set CAL_FRAMES = None to disable calibration entirely.
  CAL_RANGE_M   roughly how far the reflector sat (m), e.g. 4.0. Used to pick the
                right range bin. Set None to just grab the strongest static return.
  CAL_FILE      reuse a saved .cal.npz instead of building from a reflector (takes
                priority over CAL_FRAMES). Valid only if the RF chain wasn't reset
                between recordings -- the loader prints the full caveat.

Detection / trust thresholds (all in dB). Two are AMPLITUDE, two are CONFIDENCE:
  STORE_DB      cache is built down to this many dB below each frame's peak. Keep
                it generous (30) so DET_DB can be re-trimmed later without the .bin.
  DET_DB        AMPLITUDE display trim: keep points within this many dB of the peak.
                12-20 is right for bg-subtracted data. Render-time -> instant.
  STORE_CONF_DB cache keeps points with angle confidence >= this (low floor, 3) so
                MIN_CONF_DB can be swept later without reprocessing the .bin.
  MIN_CONF_DB   CONFIDENCE display trim: drop points whose angle peak-to-median
                sharpness is below this. Render-time now -> instant. 10-12 is a sane
                floor (8 barely filters on this grid; real targets score in the teens+).
  FLOOR_DB      skip a frame unless its peak rises this many dB over the residual.
  GATE_DROP_DB  --nobg only: keep frames within this many dB of the loudest frame.
  MIN_RANGE_M / MAX_RANGE_M  render-time RANGE gate (m). Drop points outside the
                window. OFF by default (0 / None). Range is stored per point, so this
                trims instantly from a cache -- e.g. MAX_RANGE_M=5 cuts far clutter.

  -> DET_DB and MIN_CONF_DB are independent: a point shows only if it's strong
     enough (DET_DB) AND its angle is sharp enough (MIN_CONF_DB). Note DET_DB/STORE_DB
     are 20*log10 amplitude dB; confidence is 10*log10 of a score ratio -- different
     scales, don't compare the numbers directly.

Coloring:
  COLOR_BY      "intensity" (viridis, dB) or "doppler" (diverging colormap, m/s,
                centered at 0). Velocity comes from the chirp timing below.
  VEL_SIGN      flip to -1.0 if approaching/receding colors are swapped.

Angle search field-of-view / resolution:
  AZ_SIGN       +1.0 normally; -1.0 if real targets come out mirrored left/right.
  FOV_AZ/FOV_EL believable search half-range in az / el (deg).
  AZ_STEP/EL_STEP  grid resolution (deg).

Scene / view / figure:
  XLIM / YMAX / ZLIM   plot limits (m). z is the weakest axis -- read it loosely.
  VIEW_ELEV / VIEW_AZIM  3D camera. azim=-60 -> forward into upper-right; -120 ->
                upper-left; elev ~30 balanced, ~45 looks down harder.
  FIG_SIZE / POINT_SIZE  canvas size and marker size.
  DB_RANGE  intensity color range (dB below the brightest point).

Output:
  FPS    playback frame rate.  OUT  output filename (.mp4, else .gif).
"""

import sys
import os
import mmap
import struct
import numpy as np

# ============================================================
# CONFIG  -- edit these, then just run:  python bgsub_3d.py capture.bin
# ============================================================

# ---- capture format (must match the firmware; don't touch unless it changed) ----
SAMPLES          = 256
NUM_RX           = 8
NUM_TX           = 6                       # 6 TDMA slots
NUM_LOOPS        = 64
CHIRPS_PER_FRAME = NUM_LOOPS * NUM_TX      # 384
HEADER_MAGIC     = 0xA1B2C3D4
HEADER_BYTES     = 16
MAGIC_LE         = struct.pack("<I", HEADER_MAGIC)
DATA_PER_CHIRP   = SAMPLES * NUM_RX * 4    # 8192  (int16 I + int16 Q per sample)
BLOCK_BYTES      = DATA_PER_CHIRP + HEADER_BYTES

# ---- range axis (MUST match current firmware -- stale values give 2x range errors) ----
C_LIGHT  = 299792458.0
ADC_RATE = 10000e3        # 10 Msps  (digOutSampleRate)
SLOPE    = 150.06e12      # Hz/s
F0       = 76e9           # start frequency (Hz)
LAMBDA   = C_LIGHT / F0
#   range bin spacing ~3.9 cm/bin  ->  256 bins span ~10 m (max unambiguous range)

# ---- doppler / velocity axis (from your chirp profile) ----
#   idle 7 us + ramp 28 us = 35 us per chirp; x6 TX (TDMA) = 210 us between
#   successive chirps of the SAME virtual channel -> that is the velocity sample
#   period. Over NUM_LOOPS samples this gives the per-bin velocity below.
IDLE_TIME     = 7e-6          # s  (idleTimeConst)
RAMP_END_TIME = 28e-6         # s  (rampEndTime)
T_CHIRP       = IDLE_TIME + RAMP_END_TIME          # 35 us per chirp
T_DOPPLER     = NUM_TX * T_CHIRP                   # 210 us between same-TX samples
V_PER_BIN     = LAMBDA / (2.0 * NUM_LOOPS * T_DOPPLER)   # ~0.15 m/s per doppler bin
V_MAX         = (NUM_LOOPS // 2) * V_PER_BIN             # ~+/-4.7 m/s unambiguous
VEL_SIGN      = +1.0          # flip to -1.0 if approaching/receding come out swapped

# ---- frame ranges (read off your range_over_frames.png stage map) ----
BG_FRAMES    = (1000, 2000)   # empty-scene frames -> background template
TARGET_START = 2000           # first frame to build the cloud from (moving-target stage)
CAL_FRAMES   = (100, 800)     # reflector-at-boresight frames; set None to disable cal
CAL_RANGE_M  = 4.2            # where the reflector sat (m); None = auto-pick strongest
CAL_FILE     = None           # path to a saved <capture>.cal.npz to REUSE instead of
                              #   building from CAL_FRAMES (for recordings with no
                              #   reflector). See the RF-phase caveat where it loads.
                              #   When CAL_FILE is set it takes priority over CAL_FRAMES.

# ---- detection / trust thresholds (dB) ----
STORE_DB       = 40.0         # build the point CACHE down to this many dB below peak
                              #   (stores extra weak points so DET_DB can be re-trimmed
                              #   later from the cache without reprocessing the .bin)
DET_DB         = 35.0         # DISPLAY amplitude trim: keep points within this many dB
                              #   of the frame peak. Render-time -> instant from a cache.
STORE_CONF_DB  = 3.0          # cache also keeps every point whose angle confidence is at
                              #   least this (a low floor, so MIN_CONF_DB can be swept
                              #   later from the cache without reprocessing the .bin)
MIN_CONF_DB    = 10.0         # DISPLAY confidence trim: drop points whose angle isn't at
                              #   least this sharp. Render-time now -> sweep it instantly.
                              #   ~10-12 is a sane floor; 8 barely filters on this grid.
FLOOR_DB       = 6.0          # skip a frame unless its peak is this far over residual
GATE_DROP_DB   = 30.0         # --nobg only: keep frames within this dB of loudest frame

# ---- range gate (render-time spatial trim; OFF by default so it changes nothing) ----
MIN_RANGE_M  = 0.5            # drop points closer than this (m). 0 = no near limit.
MAX_RANGE_M  = 5           # drop points farther than this (m). None = no far limit.
                              #   e.g. set MAX_RANGE_M = 5.0 to cut everything past 5 m.
                              #   Range is stored per point, so this is a render-time
                              #   trim like DET_DB/MIN_CONF_DB -- instant from a cache,
                              #   no reprocessing, no points lost from the cache.

# ---- angle search field-of-view / resolution ----
AZ_SIGN  = +1.0               # flip to -1.0 if targets come out mirrored left/right
FOV_AZ   = 60.0               # azimuth   search half-range (deg)
FOV_EL   = 30.0               # elevation search half-range (deg)
AZ_STEP  = 0.5                # azimuth grid step (deg)
EL_STEP  = 1.0                # elevation grid step (deg)

# ---- scene limits (m) + view ----
XLIM     = 3.0                # x in -XLIM .. +XLIM   (left-right)
YMAX     = 5.0               # y in 0 .. YMAX        (forward)
ZLIM     = 3.0                # z in -ZLIM .. +ZLIM   (up-down; weakest axis)
# left panel = 3D perspective. Orientation "B": forward (y) goes AWAY into the
# screen, left-right (x) across, looking down. VIEW_AZIM = -60 puts forward into the
# upper-RIGHT (clockwise ~30 deg); use -120 for the mirror (upper-LEFT). VIEW_ELEV
# tilts the camera down: ~30 is a good balance, ~45 looks down harder.
VIEW_ELEV = 25.0
VIEW_AZIM = -60.0
DB_RANGE  = 30.0              # intensity color range (dB below the brightest point)

# ---- point coloring ----
COLOR_BY  = "intensity"       # "intensity" (viridis, dB) or "doppler" (diverging, m/s)
#   intensity -> brighter = stronger return.  doppler -> color = radial velocity,
#   centered at 0 (blue/red = toward/away), spanning +/-V_MAX. Stored either way, so
#   you can switch this and re-render instantly (from the .bin OR a cache).

# ---- figure ----
FIG_SIZE   = (18, 8)          # bigger canvas (was 15x7)
POINT_SIZE = 6               # scatter marker size (smaller = less fat; was 20)

# ---- output ----
FPS = 20.0
OUT = "bgsub_3d.mp4"          # .mp4 if ffmpeg present, else auto-falls back to .gif

# ---- antenna geometry (from Fig 4-1 + firing order in common.c) ----
#   TX_POS columns: [x in half-lambda, z in lambda]; slot 2 is the lifted (elev) TX.
TX_POS = np.array([
    [0,  0.0],    # slot 0  Dev1.TX0
    [4,  0.0],    # slot 1  Dev1.TX1
    [2,  0.8],    # slot 2  Dev1.TX2   <-- lifted 0.8 lambda (elevation)
    [7,  0.0],    # slot 3  Dev2.TX0
    [11, 0.0],    # slot 4  Dev2.TX1
    [15, 0.0],    # slot 5  Dev2.TX2
], dtype=float)
RX_X = np.array([0, 1, 2, 3, 19, 20, 21, 22], dtype=float)   # half-lambda

# ============================================================
# RANGE AXIS
# ============================================================
def range_bin_to_m(bin_idx):
    dR = C_LIGHT * ADC_RATE / (2.0 * SLOPE * SAMPLES)
    return bin_idx * dR


# ============================================================
# FILE I/O  (mmap-based, so multi-GB .bin files never load into RAM)
#   Each chirp block is [8192 bytes data][16 byte header]; the magic word marks
#   the header, which sits AFTER the data, so the data is the 8192 bytes before it.
# ============================================================
def chirp_offsets(mm):
    starts = []
    i = mm.find(MAGIC_LE)
    while i != -1:
        if i >= DATA_PER_CHIRP:
            starts.append(i - DATA_PER_CHIRP)
        i = mm.find(MAGIC_LE, i + HEADER_BYTES)
    return starts


def frame_cube(mm, starts, f):
    """Frame f -> complex cube [loop, tx, rx, sample]. Chirps are contiguous,
    so frame f is just the f-th block of CHIRPS_PER_FRAME chirps."""
    sel = starts[f * CHIRPS_PER_FRAME:(f + 1) * CHIRPS_PER_FRAME]
    blk = np.empty((CHIRPS_PER_FRAME, DATA_PER_CHIRP), np.uint8)
    for k, off in enumerate(sel):
        blk[k] = np.frombuffer(mm, np.uint8, DATA_PER_CHIRP, off)
    iq = blk.view(np.int16).astype(np.float32)
    c = iq[:, 0::2] + 1j * iq[:, 1::2]                  # I + jQ
    c = c.reshape(CHIRPS_PER_FRAME, NUM_RX, SAMPLES)    # [chirp, rx, sample]
    return c.reshape(NUM_LOOPS, NUM_TX, NUM_RX, SAMPLES)  # chirp = loop*NUM_TX + tx


# ============================================================
# RANGE FFT  ->  Doppler FFT + detection
# ============================================================
def range_fft(cube):
    win = np.hanning(SAMPLES).astype(np.float32)
    return np.fft.fft(cube * win[None, None, None, :], axis=3)   # [loop, tx, rx, range]


def range_doppler(rng):
    win = np.hanning(NUM_LOOPS).astype(np.float32)
    dop = np.fft.fft(rng * win[:, None, None, None], axis=0)
    dop = np.fft.fftshift(dop, axes=0)               # [doppler, tx, rx, range]
    mag = np.abs(dop).sum(axis=(1, 2))               # [doppler, range], summed over channels
    return dop, mag


def detect(mag, db_below_peak, min_range_bin=4, max_targets=300):
    m = mag.copy()
    m[:, :min_range_bin] = 0.0                       # kill near-range coupling
    thr = m.max() * 10 ** (-db_below_peak / 20.0)
    cells = np.argwhere(m > thr)                     # rows of [doppler_bin, range_bin]
    if len(cells) == 0:
        return cells
    strength = m[cells[:, 0], cells[:, 1]]
    order = np.argsort(strength)[::-1][:max_targets]
    return cells[order]


# ============================================================
# MANUAL ANGLE SEARCH  (steering-vector / Bartlett over real element geometry)
#   For each candidate (az, el) on a grid we build the phase the actual 48 virtual
#   elements would see, correlate with the measured snapshot, keep the best match.
#   No zero-filling, no FFT-grid quantization. Returns a confidence (peak-to-median
#   sharpness in dB) so noise-only cells -- where no angle matches -- get dropped.
# ============================================================
# real element positions in lambda, one per (tx, rx) virtual channel
_xs, _zs = [], []
for _tx in range(NUM_TX):
    for _rx in range(NUM_RX):
        _xs.append((TX_POS[_tx, 0] + RX_X[_rx]) * 0.5)   # half-lambda cols -> lambda
        _zs.append(TX_POS[_tx, 1])                       # already in lambda
_xs = np.asarray(_xs)
_zs = np.asarray(_zs)                                    # (48,)

# precompute the steering dictionary ONCE (grid is fixed)
_az = np.arange(-FOV_AZ, FOV_AZ + AZ_STEP, AZ_STEP)
_el = np.arange(-FOV_EL, FOV_EL + EL_STEP, EL_STEP)
_AZ, _EL = np.meshgrid(np.radians(_az), np.radians(_el), indexing="ij")
_u = AZ_SIGN * np.sin(_AZ) * np.cos(_EL)
_w = np.sin(_EL)
_phase = 2 * np.pi * (_u[..., None] * _xs[None, None, :] +
                      _w[..., None] * _zs[None, None, :])
_STEER_C = np.conj(np.exp(1j * _phase)).reshape(-1, _xs.size)   # (ngrid, 48)


def estimate_angles_search(snap):
    """snap [NUM_TX, NUM_RX] complex -> (az_deg, el_deg, conf_db).
    Always returns the best-matching angle and its confidence (peak-to-median
    sharpness in dB). The caller decides what confidence to trust -- the cache
    stores conf so MIN_CONF_DB can be applied as a render-time trim."""
    s = snap.reshape(-1)                             # (48,)
    P = np.abs(_STEER_C @ s) ** 2                    # match score over the grid
    k = int(np.argmax(P))
    conf_db = 10.0 * np.log10(P[k] / (np.median(P) + 1e-30))
    iaz, iel = np.unravel_index(k, (_az.size, _el.size))
    return float(_az[iaz]), float(_el[iel]), float(conf_db)


def to_xyz(range_m, az_deg, el_deg):
    az, el = np.radians(az_deg), np.radians(el_deg)
    x = range_m * np.cos(el) * np.sin(az)     # left-right
    y = range_m * np.cos(el) * np.cos(az)     # forward (boresight)
    z = range_m * np.sin(el)                  # up-down
    return x, y, z


# ============================================================
# CALIBRATION  (built inline from the reflector stage of THIS capture)
#   The reflector is static, so its zero-Doppler response is averaged COHERENTLY
#   across frames (noise drops, the real phase survives). cal = conj(ref)/|ref|.
#   Built on the first half of CAL_FRAMES, validated on the second half.
# ============================================================
ZERO_DOPPLER = NUM_LOOPS // 2     # zero-velocity bin after fftshift
MIN_BIN      = 4


def _zero_doppler_avg(mm, starts, f0, f1):
    acc = np.zeros((NUM_TX, NUM_RX, SAMPLES), complex)
    n = 0
    for fr in range(f0, f1):
        dop, _ = range_doppler(range_fft(frame_cube(mm, starts, fr)))
        acc += dop[ZERO_DOPPLER]
        n += 1
    return acc / max(n, 1), n


def _find_reflector_bin(acc, range_m):
    mag_r = np.abs(acc).sum(axis=(0, 1))             # [range], summed over channels
    mag_r[:MIN_BIN] = 0.0
    if range_m is not None:
        dr = range_bin_to_m(1)
        lo = max(int((range_m - 0.5) / dr), MIN_BIN)
        hi = min(int((range_m + 0.5) / dr), SAMPLES)
        return lo + int(np.argmax(mag_r[lo:hi]))
    return int(np.argmax(mag_r))


def _fmt(a):
    return "nan" if not np.isfinite(a) else f"{a:+.1f}"


def build_calibration(mm, starts, cal_frames, range_m, n_frames):
    """Return cal [tx, rx] phase-only correction, or None if it can't be built."""
    f0, f1 = cal_frames
    f1 = min(f1, n_frames)
    if f1 - f0 < 4:
        print("calibration: not enough reflector frames -> OFF")
        return None
    mid = (f0 + f1) // 2
    acc_build, nb = _zero_doppler_avg(mm, starts, f0, mid)
    acc_test,  nt = _zero_doppler_avg(mm, starts, mid, f1)

    rbin = _find_reflector_bin(acc_build, range_m)
    ref = acc_build[:, :, rbin]                      # [tx, rx]
    cal = np.conj(ref) / np.abs(ref)                 # phase-only correction

    print(f"calibration ON  (inline)  reflector at bin {rbin} = "
          f"{range_bin_to_m(rbin):.2f} m  (built on {nb} frames, validated on {nt})")
    if range_m is not None and abs(range_bin_to_m(rbin) - range_m) > 0.6:
        print(f"  !! reflector range != the {range_m:.1f} m you set -- SLOPE/F0 may be "
              f"stale for this firmware (range axis off).")

    ph = np.angle(cal.reshape(-1), deg=True)
    print(f"  per-channel phase offsets removed: {ph.min():+.0f} .. {ph.max():+.0f} deg "
          f"(std {ph.std():.0f})")

    # non-circular check: held-out half's reflector angle, before vs after cal
    snap = acc_test[:, :, rbin]
    az0, el0, c0 = estimate_angles_search(snap)
    az1, el1, c1 = estimate_angles_search(snap * cal)
    print(f"  held-out reflector  before cal:  az={_fmt(az0)}  conf={c0:.1f} dB")
    print(f"  held-out reflector   after cal:  az={_fmt(az1)}  el={_fmt(el1)}  conf={c1:.1f} dB")
    print("  (good cal -> after-cal az near 0 deg with high confidence)")
    return cal


def save_calibration(cal, path):
    """Save the per-channel phase correction so it can be reused on other
    recordings (CAL_FILE). Stores the firmware constants it was built under so a
    mismatch can be flagged on load."""
    np.savez(path, cal=cal.astype(np.complex64),
             slope=np.float64(SLOPE), f0=np.float64(F0),
             az_sign=np.float64(AZ_SIGN), fov_az=np.float64(FOV_AZ),
             az_step=np.float64(AZ_STEP), el_step=np.float64(EL_STEP))
    print(f"  saved calibration -> {path}")


def load_calibration(path):
    """Load a saved per-channel phase correction (CAL_FILE).

    REUSE CAVEAT: this corrects FIXED per-channel phase. That's valid on another
    recording only if the RF chain phase state didn't change between captures --
    i.e. no power-cycle / sensor restart / firmware-profile change in between
    (ideally the same powered session). If it was reset, the fixed PCB/cable part
    still helps but a per-startup phase component is stale and the left/right
    mirror flip can return. The new recording has no reflector to re-validate
    against, so if azimuth looks mirrored, rebuild the cal or flip AZ_SIGN."""
    d = np.load(path, allow_pickle=False)
    cal = d["cal"].astype(complex)
    print(f"calibration LOADED from {path}  (reusing on this recording)")
    if "slope" in d and abs(float(d["slope"]) - SLOPE) > 1e6:
        print(f"  !! cal was built with SLOPE={float(d['slope']):.3e}, now {SLOPE:.3e} "
              f"-- firmware changed; cal may be invalid.")
    if "az_sign" in d and float(d["az_sign"]) != AZ_SIGN:
        print(f"  !! cal built with AZ_SIGN={float(d['az_sign']):+.0f}, now {AZ_SIGN:+.0f}.")
    print("  reuse is valid ONLY if the RF chain wasn't reset (no power-cycle / restart")
    print("  / profile change) since this cal was captured. If azimuth looks mirrored,")
    print("  the saved cal is stale -- rebuild from a reflector or flip AZ_SIGN.")
    return cal


# ============================================================
# ONE FRAME -> per-point rows
# ============================================================
def cloud(clean, cal, store_db, gate):
    """clean cube -> per-point rows, one row per detection within store_db of the
    frame peak AND with angle confidence >= STORE_CONF_DB.
    Columns: [x, y, z, amp, vel, reldB, range_m, az, el, conf].
      reldB = dB below this frame's peak (<=0)  -> DET_DB re-trims later.
      conf  = angle peak-to-median sharpness (dB) -> MIN_CONF_DB re-trims later.
      vel   = signed radial velocity (m/s).
    Empty (0x10) if the frame doesn't clear the gate."""
    rng = range_fft(clean)
    dop, mag = range_doppler(rng)
    if mag.max() < gate:
        return np.zeros((0, 10))
    peak = mag[:, 4:].max()                           # ignore near-range coupling
    pts = []
    for dbin, rbin in detect(mag, store_db):
        snap = dop[dbin, :, :, rbin]                  # [tx, rx]
        if cal is not None:
            snap = snap * cal                         # remove per-channel phase offsets
        az, el, conf = estimate_angles_search(snap)
        if conf < STORE_CONF_DB:                      # generous floor for the cache
            continue
        R = range_bin_to_m(rbin)
        x, y, z = to_xyz(R, az, el)
        amp = float(mag[dbin, rbin])
        vel = VEL_SIGN * (dbin - ZERO_DOPPLER) * V_PER_BIN
        reldB = 20.0 * np.log10(amp / (peak + 1e-12))
        pts.append((x, y, z, amp, vel, reldB, R, az, el, conf))
    return np.array(pts) if pts else np.zeros((0, 10))


def trim(p, det_db=None, min_conf=None):
    """Render-time gates: keep points within det_db of the frame peak (col 5 = reldB)
    AND with confidence >= min_conf (col 9) AND inside [MIN_RANGE_M, MAX_RANGE_M]
    (col 6 = range_m). det_db/min_conf default to the globals; the range gate reads
    the globals directly (MAX_RANGE_M = None means no far limit)."""
    if len(p) == 0:
        return p
    det_db = DET_DB if det_db is None else det_db
    min_conf = MIN_CONF_DB if min_conf is None else min_conf
    keep = (p[:, 5] >= -det_db) & (p[:, 9] >= min_conf) & (p[:, 6] >= MIN_RANGE_M)
    if MAX_RANGE_M is not None:
        keep &= (p[:, 6] <= MAX_RANGE_M)
    return p[keep]


# ============================================================
# BUILD all clouds over the target frames
# ============================================================
def build(path, nobg, nocal):
    f = open(path, "rb")
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    starts = chirp_offsets(mm)
    n_frames = len(starts) // CHIRPS_PER_FRAME
    print(f"frames in file: {n_frames}")

    # ---- calibration: --nocal  >  CAL_FILE (reuse saved)  >  CAL_FRAMES (build inline) ----
    cal = None
    if nocal:
        print("calibration OFF (--nocal)")
    elif CAL_FILE is not None:
        cal = load_calibration(CAL_FILE)
    elif CAL_FRAMES is None:
        print("calibration OFF (CAL_FRAMES=None)")
    else:
        cal = build_calibration(mm, starts, CAL_FRAMES, CAL_RANGE_M, n_frames)
        if cal is not None:
            save_calibration(cal, os.path.splitext(path)[0] + ".cal.npz")
    cal_label = "cal" if cal is not None else "no cal"

    target = min(TARGET_START, n_frames)

    # ---- background template + frame gate ----
    if nobg:
        template = np.zeros((NUM_TX, NUM_RX, SAMPLES), complex)
        peaks = []
        step = max((n_frames - target) // 80, 1)
        for fr in range(target, n_frames, step):
            _, mag = range_doppler(range_fft(frame_cube(mm, starts, fr)))
            peaks.append(mag.max())
        gate = max(peaks) * 10 ** (-GATE_DROP_DB / 20.0)
        print(f"RAW mode (no suppression)  gate {gate:.1f}  "
              f"({GATE_DROP_DB:.0f} dB below loudest frame)")
    else:
        bg0, bg1 = BG_FRAMES[0], min(BG_FRAMES[1], n_frames)
        acc = np.zeros((NUM_TX, NUM_RX, SAMPLES), complex); cnt = 0
        for fr in range(bg0, bg1):
            acc += frame_cube(mm, starts, fr).sum(axis=0); cnt += NUM_LOOPS
            if fr % 200 == 0:
                print(f"\r  template: frame {fr}", end="", flush=True)
        template = acc / max(cnt, 1)
        print(f"\r  background {bg0}-{bg1}: template from {bg1 - bg0} frames")
        peaks = []
        step = max((bg1 - bg0) // 80, 1)
        for fr in range(bg0, bg1, step):
            _, mag = range_doppler(range_fft(frame_cube(mm, starts, fr) - template[None]))
            peaks.append(mag.max())
        gate = max(peaks) * 10 ** (FLOOR_DB / 20.0)
        print(f"  gate {gate:.1f}  (+{FLOOR_DB:.0f} dB over residual)")

    # ---- per-frame clouds (stored generously down to STORE_DB; trimmed at render) ----
    clouds = []
    for fr in range(target, n_frames):
        clouds.append(cloud(frame_cube(mm, starts, fr) - template[None], cal, STORE_DB, gate))
        if fr % 50 == 0:
            print(f"\r  clouds: frame {fr}/{n_frames}", end="", flush=True)
    print()
    mm.close(); f.close()
    print(f"  {sum(1 for p in clouds if len(p))}/{len(clouds)} frames have points "
          f"(stored to {STORE_DB:.0f} dB below peak)")
    return clouds, cal_label


# ============================================================
# POINT CACHE  (store detected points so movies can be rebuilt without the .bin)
#   Saved columns: [frame, x, y, z, amp, vel, reldB, range_m, az, el, conf]
# ============================================================
def save_cache(clouds, path, cal_label):
    rows = []
    for fr, p in enumerate(clouds):
        if len(p):
            fcol = np.full((len(p), 1), fr, np.float32)
            rows.append(np.hstack([fcol, p.astype(np.float32)]))
    pts = np.vstack(rows) if rows else np.zeros((0, 11), np.float32)
    np.savez_compressed(path, points=pts,
                        n_frames=np.int64(len(clouds)),
                        cal_label=str(cal_label),
                        store_db=np.float64(STORE_DB),
                        store_conf_db=np.float64(STORE_CONF_DB),
                        v_per_bin=np.float64(V_PER_BIN))
    print(f"  saved point cache -> {path}  ({len(pts)} pts, {len(clouds)} frames, "
          f"to {STORE_DB:.0f} dB / conf {STORE_CONF_DB:.0f} dB)")


def load_cache(path):
    d = np.load(path, allow_pickle=False)
    pts = d["points"]
    n_frames = int(d["n_frames"]); cal_label = str(d["cal_label"])
    store_db = float(d["store_db"])
    store_conf = float(d["store_conf_db"]) if "store_conf_db" in d else 0.0
    clouds = [np.zeros((0, 10)) for _ in range(n_frames)]
    if len(pts):
        order = np.argsort(pts[:, 0], kind="stable")
        pts = pts[order]
        fr = pts[:, 0].astype(int)
        bnd = np.searchsorted(fr, np.arange(n_frames + 1))
        for f in range(n_frames):
            clouds[f] = pts[bnd[f]:bnd[f + 1], 1:]     # drop frame col -> 10 cols
    print(f"  loaded cache {path}: {len(pts)} pts, {n_frames} frames, "
          f"stored to {store_db:.0f} dB / conf {store_conf:.0f} dB  ({cal_label})")
    if DET_DB > store_db + 1e-6:
        print(f"  !! DET_DB={DET_DB:.0f} is looser than the {store_db:.0f} dB stored -- "
              f"weaker points aren't in the cache; reprocess the .bin to keep them.")
    if MIN_CONF_DB < store_conf - 1e-6:
        print(f"  !! MIN_CONF_DB={MIN_CONF_DB:.0f} is below the {store_conf:.0f} dB conf "
              f"floor the cache was built with; reprocess the .bin to go lower.")
    return clouds, cal_label



def render(clouds, cal_label, out):
    """Side-by-side, synchronized: LEFT = 3D perspective (orientation B), RIGHT =
    top-down (x vs y). Points are trimmed to DET_DB at render time and colored by
    COLOR_BY ('intensity' or 'doppler'). Both panels share the frame index."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

    doppler = (COLOR_BY == "doppler")
    if doppler:
        cmap, vmin, vmax, clabel = "viridis", -V_MAX, +V_MAX, "radial velocity (m/s)"
    else:
        amps = [p[:, 3] for p in clouds if len(p)]
        vmax = float(20 * np.log10(np.concatenate(amps).max() + 1e-6)) if amps else 0.0
        vmin = vmax - DB_RANGE
        cmap, clabel = "viridis", "intensity (dB)"

    def color_of(p):
        return p[:, 4] if doppler else 20 * np.log10(p[:, 3] + 1e-6)

    fig = plt.figure(figsize=FIG_SIZE)

    # ---- LEFT: 3D perspective view ----
    ax3 = fig.add_subplot(121, projection="3d")
    scat3 = ax3.scatter([0], [0], [0], c=[vmin], cmap=cmap,
                        vmin=vmin, vmax=vmax, s=POINT_SIZE, depthshade=False)
    ax3.set_xlim(-XLIM, XLIM); ax3.set_ylim(0, YMAX); ax3.set_zlim(-ZLIM, ZLIM)
    ax3.set_xlabel("x  left-right (m)")
    ax3.set_ylabel("y  forward (m)")
    ax3.set_zlabel("z  up-down (m)")
    try:
        ax3.set_box_aspect((2 * XLIM, YMAX, 2 * ZLIM))
    except Exception:
        pass
    ax3.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)
    if VIEW_ELEV == 0:
        ax3.set_xticklabels([])   # x is edge-on in a pure side view; labels just clutter
    ax3.set_title(f"perspective ({cal_label})")

    # ---- RIGHT: top-down (bird's eye) ----
    ax2 = fig.add_subplot(122)
    scat2 = ax2.scatter([0], [0], c=[vmin], cmap=cmap,
                        vmin=vmin, vmax=vmax, s=POINT_SIZE)
    ax2.set_xlim(-XLIM, XLIM); ax2.set_ylim(0, YMAX)
    ax2.set_xlabel("x  left-right (m)")
    ax2.set_ylabel("y  forward (m)")
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(alpha=0.25)
    ax2.set_title("top down")

    fig.colorbar(scat2, ax=[ax3, ax2], label=clabel, shrink=0.6)
    sup = fig.suptitle("")

    def update(i):
        p = trim(clouds[i], DET_DB, MIN_CONF_DB)
        if len(p):
            c = color_of(p)
            scat3._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
            scat3.set_array(c)
            scat2.set_offsets(np.column_stack((p[:, 0], p[:, 1])))
            scat2.set_array(c)
        else:
            scat3._offsets3d = ([], [], [])
            scat3.set_array(np.array([]))
            scat2.set_offsets(np.empty((0, 2)))
            scat2.set_array(np.array([]))
        rg = ""
        if MIN_RANGE_M > 0 or MAX_RANGE_M is not None:
            lo = f"{MIN_RANGE_M:.1f}"; hi = "inf" if MAX_RANGE_M is None else f"{MAX_RANGE_M:.1f}"
            rg = f", range {lo}-{hi}m"
        sup.set_text(f"frame {i + 1}/{len(clouds)}   {len(p)} pts   "
                     f"({cal_label}, {COLOR_BY}, {DET_DB:.0f}dB / conf {MIN_CONF_DB:.0f}dB{rg})")
        return scat3, scat2

    anim = FuncAnimation(fig, update, frames=len(clouds), interval=1000.0 / FPS, blit=False)
    writer = None
    if out.lower().endswith(".mp4"):
        try:
            import imageio_ffmpeg
            matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        except Exception:
            pass
        writer = FFMpegWriter(fps=FPS) if FFMpegWriter.isAvailable() else None
        if writer is None:
            out = out[:-4] + ".gif"
            print("  ffmpeg not found -> GIF:", out, "(pip install imageio-ffmpeg for mp4)")
    if writer is None:
        writer = PillowWriter(fps=FPS)
    print(f"  encoding {out} ...", flush=True)
    anim.save(out, writer=writer); plt.close(fig); print("  saved", out)


# ============================================================
# SELF TEST: synthesise a known target, run the whole pipeline, check recovery
# ============================================================
def _synth_cube(R_m, az_deg, el_deg, snr_db=30):
    az, el = np.radians(az_deg), np.radians(el_deg)
    f_beat = 2 * SLOPE * R_m / C_LIGHT
    n = np.arange(SAMPLES)
    range_sig = np.exp(2j * np.pi * f_beat * n / ADC_RATE)   # beat tone -> range bin
    cube = np.zeros((NUM_LOOPS, NUM_TX, NUM_RX, SAMPLES), complex)
    for tx in range(NUM_TX):
        x_tx, z_tx = TX_POS[tx]
        for rx in range(NUM_RX):
            x_lam = (x_tx + RX_X[rx]) * 0.5
            z_lam = z_tx
            # match the steering convention (AZ_SIGN) so the test is self-consistent
            phase = 2 * np.pi * (AZ_SIGN * x_lam * np.sin(az) * np.cos(el) +
                                 z_lam * np.sin(el))
            cube[:, tx, rx, :] = range_sig[None, :] * np.exp(1j * phase)
    noise = np.random.randn(*cube.shape) + 1j * np.random.randn(*cube.shape)
    return cube + noise * 10 ** (-snr_db / 20)


def selftest():
    print("self-test: target at R=6.0 m, az=+20 deg, el=+8 deg")
    cube = _synth_cube(6.0, 20.0, 8.0)
    pts = cloud(cube, cal=None, store_db=STORE_DB, gate=0.0)
    if len(pts) == 0:
        print("  RESULT: CHECK  (no points detected)")
        return False
    best = pts[np.argmax(pts[:, 3])]
    x, y, z = best[0], best[1], best[2]
    R = float(np.sqrt(x * x + y * y + z * z))
    az = float(np.degrees(np.arctan2(x, y)))
    el = float(np.degrees(np.arcsin(np.clip(z / max(R, 1e-9), -1, 1))))
    print(f"  recovered  R={R:5.2f} m   az={az:+6.2f} deg   el={el:+6.2f} deg")
    print(f"  xyz        x={x:+5.2f}  y={y:+5.2f}  z={z:+5.2f}  (m)")
    print(f"  velocity per doppler bin = {V_PER_BIN:.3f} m/s, unambiguous +/-{V_MAX:.2f} m/s")
    ok = abs(R - 6.0) < 0.15 and abs(az - 20) < 2.0 and abs(el - 8) < 3.0
    print("  RESULT:", "PASS" if ok else "CHECK")

    # pure noise should be trimmed by the confidence gate
    noise = np.random.randn(NUM_TX, NUM_RX) + 1j * np.random.randn(NUM_TX, NUM_RX)
    a, e, c = estimate_angles_search(noise)
    print(f"  noise snapshot -> conf={c:.1f} dB  "
          f"{'trimmed by MIN_CONF_DB' if c < MIN_CONF_DB else 'KEPT (raise MIN_CONF_DB)'}")
    return ok


# ============================================================
def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        return
    src = args[0]
    nobg = "--nobg" in sys.argv
    nocal = "--nocal" in sys.argv

    if src.lower().endswith(".npz"):
        # ---- replay from a saved point cache (no .bin needed) ----
        clouds, cal_label = load_cache(src)
        out = str(args[1]) if len(args) > 1 else OUT
        render(clouds, cal_label, out)
    else:
        # ---- process the .bin: build clouds, save the cache, then render ----
        clouds, cal_label = build(src, nobg=nobg, nocal=nocal)
        cache_path = os.path.splitext(src)[0] + ".cache.npz"
        save_cache(clouds, cache_path, cal_label)
        out = OUT
        if nobg and out == "bgsub_3d.mp4":
            out = "raw_3d.mp4"
        render(clouds, cal_label, out)


if __name__ == "__main__":
    main()
