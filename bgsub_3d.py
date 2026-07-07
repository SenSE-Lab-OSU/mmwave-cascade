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
    python bgsub_3d.py capture.bin --nocomp     # TDM Doppler compensation OFF (A/B the effect)
    python bgsub_3d.py capture.bin --window     # sliding-window temporal point accumulation
    python bgsub_3d.py capture.cache.npz --window   # ...or sweep the window from a cache
    python bgsub_3d.py capture.bin --elev       # board rotated 90deg: swap az<->el resolution
    python bgsub_3d.py --selftest --elev        # check the math in the rotated geometry
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
The only command-line switches are: --nocal, --nobg, --nocomp, --window/--nowindow,
--elev/--noelev, --selftest.

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

Temporal aggregation (sliding window; OFF by default, --window to enable):
  WINDOW_AGG / WINDOW_FRAMES  each output frame shows the UNION of the detected
                points from a window of +/-WINDOW_FRAMES frames around it, so a
                sparse per-frame cloud accumulates into a denser one. The movie
                keeps the SAME output frames; only point density changes. It is a
                render-time knob (sweep it from a cache). Building the .bin with
                --window also stores WINDOW_FRAMES "pre-roll" frames before
                TARGET_START so the first output frame can look back past it.
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
ADC_START = 1.5e-6                                   # adcStartTimeConst in common.c
ACQ_TIME  = SAMPLES / ADC_RATE                       # 25.6 us
F_CENTER  = F0 + SLOPE * (ADC_START + ACQ_TIME / 2)  # ~78.15 GHz (ADC-window center)
LAMBDA    = C_LIGHT / F_CENTER
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
BG_FRAMES    = (1050, 1350)   # empty-scene frames -> background template
TARGET_START = 1500           # first frame to build the cloud from (moving-target stage)
CAL_FRAMES   = (300, 600)     # reflector-at-boresight frames; set None to disable cal
CAL_RANGE_M  = 4.4            # where the reflector sat (m); None = auto-pick strongest
CAL_FILE     = None           # path to a saved <capture>.cal.npz to REUSE instead of
                              #   building from CAL_FRAMES (for recordings with no
                              #   reflector). See the RF-phase caveat where it loads.
                              #   When CAL_FILE is set it takes priority over CAL_FRAMES.

# ---- detection / trust thresholds (dB) ----
STORE_DB       = 40.0         # build the point CACHE down to this many dB below peak
                              #   (stores extra weak points so DET_DB can be re-trimmed
                              #   later from the cache without reprocessing the .bin)
DET_DB         = 20.0         # DISPLAY amplitude trim: keep points within this many dB
                              #   of the frame peak. Render-time -> instant from a cache.
STORE_CONF_DB  = 3.0          # cache also keeps every point whose angle confidence is at
                              #   least this (a low floor, so MIN_CONF_DB can be swept
                              #   later from the cache without reprocessing the .bin)
MIN_CONF_DB    = 12.0         # DISPLAY confidence trim: drop points whose angle isn't at
                              #   least this sharp. Render-time now -> sweep it instantly.
                              #   ~10-12 is a sane floor; 8 barely filters on this grid.
FLOOR_DB       = 6.0          # skip a frame unless its peak is this far over residual
GATE_DROP_DB   = 30.0         # --nobg only: keep frames within this dB of loudest frame

# ---- range gate (render-time spatial trim; OFF by default so it changes nothing) ----
MIN_RANGE_M  = 0.5            # drop points closer than this (m). 0 = no near limit.
MAX_RANGE_M  = 6           # drop points farther than this (m). None = no far limit.
                              #   e.g. set MAX_RANGE_M = 5.0 to cut everything past 5 m.
                              #   Range is stored per point, so this is a render-time
                              #   trim like DET_DB/MIN_CONF_DB -- instant from a cache,
                              #   no reprocessing, no points lost from the cache.

# ---- temporal aggregation (sliding window; render-time, OFF by default) ----
WINDOW_AGG    = True  # True -> each output frame shows the UNION of the detected points
                       #   from a sliding window of frames centered on it (temporal
                       #   accumulation -> denser cloud). The movie keeps the SAME output
                       #   frames (TARGET_START..end); only point density changes. Toggle
                       #   per-run with --window / --nowindow (flags override this).
WINDOW_FRAMES = 3     # half-width: aggregate this many frames BEFORE and AFTER each frame
                       #   (2*WINDOW_FRAMES+1 frames per output frame). RENDER-TIME knob --
                       #   sweep it instantly from a cache, up to the pre-roll depth the
                       #   cache was built with (see below).
                       #   BOUNDARY: so the first output frame (TARGET_START) can still look
                       #   BACK, building the .bin with --window also processes WINDOW_FRAMES
                       #   "pre-roll" frames before TARGET_START and stores them; they feed
                       #   the window but are NOT rendered as extra movie frames. The file
                       #   end has no post-roll, so the last frames clamp to a shorter window.
                       #   CAVEAT: if pre-roll reaches into an occupied stage (reflector or
                       #   target present before TARGET_START) those points bleed into the
                       #   first output frames -- keep WINDOW_FRAMES within the empty gap
                       #   ahead of TARGET_START.

# ---- angle search field-of-view / resolution ----
AZ_SIGN  = +1.0               # flip to -1.0 if targets come out mirrored left/right

# ---- elevation mode (board rotated 90 deg CCW about boresight, viewed from behind) ----
ELEV_MODE = False             # True (or --elev): the physical board is rotated 90 deg about
                              #   the boresight axis. The big azimuth ULA then becomes the
                              #   ELEVATION aperture (fine el) and the single lifted TX becomes
                              #   the AZIMUTH baseline (coarse az). This is a TRADE, not a free
                              #   win: elevation resolution improves, azimuth resolution drops.
                              #   Implemented by rotating the ARRAY MANIFOLD (not the output
                              #   points): az/el and to_xyz stay in the world frame, so all
                              #   three view panels remain correct and NO point-cloud shift is
                              #   needed. Toggle per-run with --elev / --noelev.
ELEV_SIGN = +1.0              # +1.0 = 90 deg CCW (x,z)->(-z,+x). Flip to -1.0 if the rotated
                              #   view comes out upside-down (i.e. the +90/-90 ambiguity) --
                              #   confirm on real data with a target of known height, don't
                              #   assume. (Analogous to AZ_SIGN for the left/right mirror.)

FOV_AZ   = 75.0               # azimuth   search half-range (deg)
FOV_EL   = 10.0               # elevation search half-range (deg)
AZ_STEP  = 0.5                # azimuth grid step (deg)
EL_STEP  = 1.0                # elevation grid step (deg)

# ---- scene limits (m) + view ----
XLIM     = 3.0                # x in -XLIM .. +XLIM   (left-right)
YMAX     = 6.0               # y in 0 .. YMAX        (forward)
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
FIG_SIZE   = (22, 7)          # canvas for the 3-panel layout (3D | x-z | x-y)
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

# ---- LAYOUT-vs-RF wavelength normalization (fixes a ~2% angular SCALE error) ----
#   TX_POS / RX_X above are FIXED MILLIMETER spacings written as multiples of the board's
#   LAYOUT design wavelength (lambda_layout) -- NOT of the current chirp. The steering phase
#   2*pi*(u*x + w*z) needs the positions in multiples of the CURRENT RF wavelength (LAMBDA,
#   the ADC-window-center value defined above). Those two wavelengths differ, so every
#   coordinate must be multiplied by GEOMETRY_SCALE = lambda_layout / lambda_rf.
#
#   Omitting it makes the beamformer return  sin(theta_est) = GEOMETRY_SCALE * sin(theta_true)
#   -- a systematic angular scale error (~2% here; zero at boresight, growing with angle).
#   NOTE this is invisible to two things people trust: a boresight calibration reflector
#   (zero phase slope at boresight regardless of scale) and the synthetic self-test (it
#   generates AND recovers with the same _XLAM/_ZLAM, so a common scale cancels).
#
#   LAMBDA_LAYOUT comes from a fixed TI-drawing dimension -- CONFIRM it against your layout;
#   the whole size of the correction depends on it. Here: 2*lambda_layout = 7.837 mm.
LAMBDA_LAYOUT  = 7.837e-3 / 2.0                  # 3.9185 mm  (layout design wavelength, ~76.5 GHz)
GEOMETRY_SCALE = LAMBDA_LAYOUT / LAMBDA           # ~1.0214 for this waveform (LAMBDA = lambda_rf)

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
# the search grid and steering dictionary are built by configure_geometry() below,
# because elevation mode changes BOTH (it swaps which angle is the fine axis).


def _base_element_positions():
    """Per-(tx, rx) virtual-element positions in lambda for the UNROTATED board:
    x = azimuth (0..18.5 lambda ULA), z = elevation (0 except the +0.8 lambda lifted TX).
    Positions are rescaled by GEOMETRY_SCALE so they are in CURRENT-RF-wavelength units,
    not layout-wavelength units (see the note by LAMBDA_LAYOUT)."""
    xs = np.empty((NUM_TX, NUM_RX))
    zs = np.empty((NUM_TX, NUM_RX))
    for tx in range(NUM_TX):
        for rx in range(NUM_RX):
            xs[tx, rx] = (TX_POS[tx, 0] + RX_X[rx]) * 0.5 * GEOMETRY_SCALE  # ->lambda_rf
            zs[tx, rx] = TX_POS[tx, 1] * GEOMETRY_SCALE                      # ->lambda_rf
    return xs, zs


def configure_geometry(elev_mode):
    """Build the search grid AND the steering dictionary for the current board
    orientation. Sets the module globals used downstream: the grid (_az/_el/_AZ/_EL),
    the element positions (_XLAM/_ZLAM per-(tx,rx), _xs/_zs flattened), and _STEER_C.

    Elevation mode does two coupled things, both handled here so the caller never has
    to touch a constant:
      1. Rotates the array 90 deg about boresight -- (x, z) -> (-z, +x) -- so the big
         18.5-lambda ULA moves onto elevation and the lifted TX onto azimuth.
      2. Swaps the search grid to follow that: the FINE axis (now elevation) inherits
         FOV_AZ/AZ_STEP, the COARSE axis (now azimuth) inherits FOV_EL/EL_STEP. Your
         az/el tunings just ride along with the resolution -- no new numbers, and the
         coarse azimuth stays inside FOV_EL (30 deg < the 0.8-lambda unambiguous ~39 deg)."""
    global _az, _el, _AZ, _EL, _XLAM, _ZLAM, _xs, _zs, _STEER_C

    if elev_mode:
        fov_az, az_step, fov_el, el_step = FOV_EL, EL_STEP, FOV_AZ, AZ_STEP
    else:
        fov_az, az_step, fov_el, el_step = FOV_AZ, AZ_STEP, FOV_EL, EL_STEP
    _az = np.arange(-fov_az, fov_az + az_step, az_step)
    _el = np.arange(-fov_el, fov_el + el_step, el_step)
    _AZ, _EL = np.meshgrid(np.radians(_az), np.radians(_el), indexing="ij")

    xs, zs = _base_element_positions()
    if elev_mode:
        xs, zs = (-ELEV_SIGN * zs), (ELEV_SIGN * xs)   # 90 deg CCW about boresight
    _XLAM, _ZLAM = xs, zs
    _xs, _zs = xs.reshape(-1), zs.reshape(-1)           # (48,)

    _u = AZ_SIGN * np.sin(_AZ) * np.cos(_EL)
    _w = np.sin(_EL)
    _phase = 2 * np.pi * (_u[..., None] * _xs[None, None, :] +
                          _w[..., None] * _zs[None, None, :])
    _STEER_C = np.conj(np.exp(1j * _phase)).reshape(-1, _xs.size)   # (ngrid, 48)


# build once at import for the ELEV_MODE default; main() rebuilds if --elev/--noelev overrides
configure_geometry(ELEV_MODE)


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
def cloud(clean, cal, store_db, gate, comp_on=True):
    """clean cube -> per-point rows, one row per detection within store_db of the
    frame peak AND with angle confidence >= STORE_CONF_DB.
    Columns: [x, y, z, amp, vel, reldB, range_m, az, el, conf].
      reldB = dB below this frame's peak (<=0)  -> DET_DB re-trims later.
      conf  = angle peak-to-median sharpness (dB) -> MIN_CONF_DB re-trims later.
      vel   = signed radial velocity (m/s).
    comp_on : apply the TDM Doppler phase compensation (True). --nocomp sets it
      False so you can A/B the effect on identical data. NOTE the compensation
      happens HERE, at build time, so --nocomp only matters when processing a .bin;
      a cache already has it baked in (the cache's label records which way).
    Empty (0x10) if the frame doesn't clear the gate."""
    rng = range_fft(clean)
    dop, mag = range_doppler(rng)
    if mag.max() < gate:
        return np.zeros((0, 10))
    peak = mag[:, 4:].max()                           # ignore near-range coupling
    pts = []
    for dbin, rbin in detect(mag, store_db):
        # ---- TDM-MIMO Doppler phase compensation (BEFORE beamforming) ----
        # In TDMA, TX m is sampled m*T_CHIRP into each loop, so a moving target
        # carries an intra-loop phase 2*pi*k*m/(NUM_LOOPS*NUM_TX) at Doppler bin k.
        # The Doppler FFT does NOT remove it (it doesn't vary loop-to-loop), so it
        # sits as a linear phase ramp across the TX columns -- indistinguishable from
        # an angle ramp, biasing az/el in proportion to velocity. De-rotate each TX
        # column by the conjugate. At k=0 (static clutter / reflector) this is a no-op.
        snap = dop[dbin, :, :, rbin]                  # [tx, rx]
        if comp_on:
            k = dbin - ZERO_DOPPLER                   # signed Doppler bin (0 = static)
            comp = np.exp(-1j * 2 * np.pi * k * np.arange(NUM_TX) / (NUM_LOOPS * NUM_TX))
            snap = snap * comp[:, None]               # TX axis de-rotated
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


def aggregate(clouds, out_idx, pre_roll, half_w):
    """Points feeding OUTPUT frame out_idx (0-based over output frames only).
    Output frame out_idx maps to source clouds index (pre_roll + out_idx); this
    unions the raw rows of the sliding window [-half_w, +half_w] around it. The
    window clamps to the available clouds -- the front reaches into the pre_roll
    lead-in that build() stored ahead of TARGET_START, the back stops at the file
    end (last frames get a shorter, one-sided window). Each row keeps its own
    columns (reldB is relative to its OWN source frame), so trim() applies to the
    union exactly as it would per frame. half_w=0 reduces to the single frame."""
    c = pre_roll + out_idx
    lo = max(c - half_w, 0)
    hi = min(c + half_w, len(clouds) - 1)
    parts = [clouds[j] for j in range(lo, hi + 1) if len(clouds[j])]
    return np.vstack(parts) if parts else np.zeros((0, 10))


# ============================================================
# BUILD all clouds over the target frames
# ============================================================
def build(path, nobg, nocal, nocomp=False, window=False):
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
    cal_label = (("cal" if cal is not None else "no cal") + (", NO COMP" if nocomp else ", comp")
                 + (", ELEV" if ELEV_MODE else ""))
    if nocomp:
        print("TDM Doppler compensation OFF (--nocomp) -- A/B mode, expect moving "
              "targets to smear/hop in angle")

    target = min(TARGET_START, n_frames)
    # ---- sliding-window pre-roll: process WINDOW_FRAMES frames BEFORE target so the
    #      first output frame can still look backward (clamped to frame 0). These extra
    #      leading clouds feed the render-time window; they are NOT rendered themselves. ----
    pre_roll  = min(WINDOW_FRAMES, target) if window else 0
    pre_start = target - pre_roll
    if window:
        print(f"temporal window ON  +/-{WINDOW_FRAMES} frames (applied at render); building "
              f"{pre_roll} pre-roll frame(s) from {pre_start} so output frame {target} "
              f"(TARGET_START) can look before it")

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
    #      starts at pre_start (= target unless --window added a pre-roll lead-in) ----
    clouds = []
    for fr in range(pre_start, n_frames):
        clouds.append(cloud(frame_cube(mm, starts, fr) - template[None], cal, STORE_DB, gate,
                            comp_on=not nocomp))
        if fr % 50 == 0:
            print(f"\r  clouds: frame {fr}/{n_frames}", end="", flush=True)
    print()
    mm.close(); f.close()
    print(f"  {sum(1 for p in clouds if len(p))}/{len(clouds)} frames have points "
          f"(stored to {STORE_DB:.0f} dB below peak)")
    return clouds, cal_label, pre_roll


# ============================================================
# POINT CACHE  (store detected points so movies can be rebuilt without the .bin)
#   Saved columns: [frame, x, y, z, amp, vel, reldB, range_m, az, el, conf]
# ============================================================
def save_cache(clouds, path, cal_label, pre_roll=0):
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
                        v_per_bin=np.float64(V_PER_BIN),
                        pre_roll=np.int64(pre_roll),
                        window_frames=np.int64(WINDOW_FRAMES))
    print(f"  saved point cache -> {path}  ({len(pts)} pts, {len(clouds)} frames"
          f"{f', {pre_roll} pre-roll' if pre_roll else ''}, "
          f"to {STORE_DB:.0f} dB / conf {STORE_CONF_DB:.0f} dB)")


def load_cache(path):
    d = np.load(path, allow_pickle=False)
    pts = d["points"]
    n_frames = int(d["n_frames"]); cal_label = str(d["cal_label"])
    store_db = float(d["store_db"])
    store_conf = float(d["store_conf_db"]) if "store_conf_db" in d else 0.0
    pre_roll = int(d["pre_roll"]) if "pre_roll" in d else 0
    clouds = [np.zeros((0, 10)) for _ in range(n_frames)]
    if len(pts):
        order = np.argsort(pts[:, 0], kind="stable")
        pts = pts[order]
        fr = pts[:, 0].astype(int)
        bnd = np.searchsorted(fr, np.arange(n_frames + 1))
        for f in range(n_frames):
            clouds[f] = pts[bnd[f]:bnd[f + 1], 1:]     # drop frame col -> 10 cols
    print(f"  loaded cache {path}: {len(pts)} pts, {n_frames} frames"
          f"{f' ({pre_roll} pre-roll)' if pre_roll else ''}, "
          f"stored to {store_db:.0f} dB / conf {store_conf:.0f} dB  ({cal_label})")
    if DET_DB > store_db + 1e-6:
        print(f"  !! DET_DB={DET_DB:.0f} is looser than the {store_db:.0f} dB stored -- "
              f"weaker points aren't in the cache; reprocess the .bin to keep them.")
    if MIN_CONF_DB < store_conf - 1e-6:
        print(f"  !! MIN_CONF_DB={MIN_CONF_DB:.0f} is below the {store_conf:.0f} dB conf "
              f"floor the cache was built with; reprocess the .bin to go lower.")
    return clouds, cal_label, pre_roll



def render(clouds, cal_label, out, pre_roll=0, half_w=0):
    """Three synchronized panels sharing the frame index: LEFT = 3D perspective
    (orientation B), MIDDLE = front view (x-z, looking down boresight), RIGHT =
    top-down (x-y). Points are trimmed to DET_DB at render time and colored by
    COLOR_BY ('intensity' or 'doppler').
    pre_roll : leading clouds that are window lead-in only, not rendered as frames.
    half_w   : sliding-window half-width; each output frame unions +/-half_w frames
               around it (0 = single frame, i.e. the original behavior)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

    n_out = len(clouds) - pre_roll        # rendered frames (pre-roll is window feed only)

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
    # 3 panels in a row: 3D | front (x-z) | top-down (x-y). The 3D panel gets more
    # width (it isn't square); the two 2D panels are equal-aspect so they render as
    # squares and don't need the extra room.
    gs = fig.add_gridspec(1, 3, width_ratios=[1.4, 1, 1])

    # ---- LEFT: 3D perspective view ----
    ax3 = fig.add_subplot(gs[0], projection="3d")
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

    # ---- MIDDLE: front view (x-z), looking down boresight; x across, z up ----
    ax_xz = fig.add_subplot(gs[1])
    scat_xz = ax_xz.scatter([0], [0], c=[vmin], cmap=cmap,
                            vmin=vmin, vmax=vmax, s=POINT_SIZE)
    ax_xz.set_xlim(-XLIM, XLIM); ax_xz.set_ylim(-ZLIM, ZLIM)
    ax_xz.set_xlabel("x  left-right (m)")
    ax_xz.set_ylabel("z  up-down (m)")
    ax_xz.set_aspect("equal", adjustable="box")
    ax_xz.grid(alpha=0.25)
    ax_xz.set_title("front (x-z)")

    # ---- RIGHT: top-down (bird's eye), x across, y forward ----
    ax2 = fig.add_subplot(gs[2])
    scat2 = ax2.scatter([0], [0], c=[vmin], cmap=cmap,
                        vmin=vmin, vmax=vmax, s=POINT_SIZE)
    ax2.set_xlim(-XLIM, XLIM); ax2.set_ylim(0, YMAX)
    ax2.set_xlabel("x  left-right (m)")
    ax2.set_ylabel("y  forward (m)")
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(alpha=0.25)
    ax2.set_title("top down (x-y)")

    fig.colorbar(scat2, ax=[ax3, ax_xz, ax2], label=clabel, shrink=0.6)
    sup = fig.suptitle("")

    def update(i):
        p = trim(aggregate(clouds, i, pre_roll, half_w), DET_DB, MIN_CONF_DB)
        if len(p):
            c = color_of(p)
            scat3._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
            scat3.set_array(c)
            scat_xz.set_offsets(np.column_stack((p[:, 0], p[:, 2])))
            scat_xz.set_array(c)
            scat2.set_offsets(np.column_stack((p[:, 0], p[:, 1])))
            scat2.set_array(c)
        else:
            scat3._offsets3d = ([], [], [])
            scat3.set_array(np.array([]))
            scat_xz.set_offsets(np.empty((0, 2)))
            scat_xz.set_array(np.array([]))
            scat2.set_offsets(np.empty((0, 2)))
            scat2.set_array(np.array([]))
        rg = ""
        if MIN_RANGE_M > 0 or MAX_RANGE_M is not None:
            lo = f"{MIN_RANGE_M:.1f}"; hi = "inf" if MAX_RANGE_M is None else f"{MAX_RANGE_M:.1f}"
            rg = f", range {lo}-{hi}m"
        wg = f", win +/-{half_w}f" if half_w > 0 else ""
        sup.set_text(f"frame {i + 1}/{n_out}   {len(p)} pts   "
                     f"({cal_label}, {COLOR_BY}, {DET_DB:.0f}dB / conf {MIN_CONF_DB:.0f}dB{rg}{wg})")
        return scat3, scat_xz, scat2

    anim = FuncAnimation(fig, update, frames=n_out, interval=1000.0 / FPS, blit=False)
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
def _synth_cube(R_m, az_deg, el_deg, vel_ms=0.0, snr_db=30):
    """Synthesise a point target at (R, az, el) moving at vel_ms (radial, m/s).
    A NONZERO vel_ms injects BOTH the inter-loop Doppler phase (so the target lands
    in a real Doppler bin) AND the intra-loop per-TX phase that TDMA introduces --
    i.e. it reproduces the exact contamination that cloud()'s Doppler compensation
    removes. vel_ms=0 reduces to the old static target (compensation is a no-op)."""
    az, el = np.radians(az_deg), np.radians(el_deg)
    f_beat = 2 * SLOPE * R_m / C_LIGHT
    n = np.arange(SAMPLES)
    range_sig = np.exp(2j * np.pi * f_beat * n / ADC_RATE)   # beat tone -> range bin
    f_d = 2.0 * vel_ms / LAMBDA                              # Doppler frequency (Hz)

    # angle phase per (tx, rx) virtual element -- uses the SAME configured geometry
    # as the search (rotated when ELEV_MODE) so the self-test validates the manifold
    # actually in use, not just the unrotated board.
    x_lam = _XLAM                                                # (tx, rx), lambda
    z_lam = _ZLAM                                                # (tx, rx), lambda
    ang = 2 * np.pi * (AZ_SIGN * x_lam * np.sin(az) * np.cos(el) +
                       z_lam * np.sin(el))                       # (tx, rx)

    # TDMA sample time of each (loop, tx): TX m fires m*T_CHIRP into a loop of length
    # T_DOPPLER (= NUM_TX*T_CHIRP). The m*T_CHIRP part is exactly what biases angle.
    t = (np.arange(NUM_LOOPS)[:, None] * T_DOPPLER +
         np.arange(NUM_TX)[None, :] * T_CHIRP)                  # (loop, tx), seconds
    dopp = 2 * np.pi * f_d * t                                  # (loop, tx)

    phase = ang[None, :, :] + dopp[:, :, None]                  # (loop, tx, rx)
    cube = range_sig[None, None, None, :] * np.exp(1j * phase)[..., None]
    noise = np.random.randn(*cube.shape) + 1j * np.random.randn(*cube.shape)
    return cube + noise * 10 ** (-snr_db / 20)


def _recover(pts):
    """Strongest point of a cloud -> (R, az, el) in m / deg."""
    best = pts[np.argmax(pts[:, 3])]
    x, y, z = best[0], best[1], best[2]
    R = float(np.sqrt(x * x + y * y + z * z))
    az = float(np.degrees(np.arctan2(x, y)))
    el = float(np.degrees(np.arcsin(np.clip(z / max(R, 1e-9), -1, 1))))
    return R, az, el


def selftest():
    ok_all = True
    print(f"velocity per doppler bin = {V_PER_BIN:.3f} m/s, unambiguous +/-{V_MAX:.2f} m/s")
    # Put the tightly-checked angle on whichever axis the CURRENT geometry resolves
    # well: azimuth for the normal board, elevation once ELEV_MODE has rotated it.
    # The coarse axis (single 0.8-lambda baseline) gets a loose tolerance -- a wide
    # peak there is expected physics, not a bug.
    if ELEV_MODE:
        AZ_T, EL_T, AZ_TOL, EL_TOL = 5.0, 20.0, 12.0, 3.0
        print("geometry: ELEVATION mode (board rotated 90 deg) -- elevation is the fine axis\n")
    else:
        AZ_T, EL_T, AZ_TOL, EL_TOL = 20.0, 8.0, 2.0, 3.0
        print("geometry: azimuth mode -- azimuth is the fine axis\n")

    # (A) STATIC target: compensation is a no-op here (k=0) -> checks geometry only.
    print(f"self-test A: STATIC target   R=6.0 m  az={AZ_T:+.0f}  el={EL_T:+.0f}")
    pts = cloud(_synth_cube(6.0, AZ_T, EL_T, vel_ms=0.0), cal=None, store_db=STORE_DB, gate=0.0)
    if len(pts) == 0:
        print("  RESULT: CHECK  (no points detected)"); return False
    R, az, el = _recover(pts)
    print(f"  recovered  R={R:5.2f} m   az={az:+6.2f}   el={el:+6.2f}")
    okA = abs(R - 6.0) < 0.15 and abs(az - AZ_T) < AZ_TOL and abs(el - EL_T) < EL_TOL
    print("  RESULT:", "PASS" if okA else "CHECK"); ok_all &= okA

    # (B) MOVING target at the SAME angle: the TDM Doppler comp must hold az/el put.
    #     Delete the comp block in cloud() and THIS is the test that fails.
    print(f"\nself-test B: MOVING target   v=+4.5 m/s  R=6.0 m  az={AZ_T:+.0f}  el={EL_T:+.0f}  (exercises TDM comp)")
    pts = cloud(_synth_cube(6.0, AZ_T, EL_T, vel_ms=4.5), cal=None, store_db=STORE_DB, gate=0.0)
    if len(pts) == 0:
        print("  RESULT: CHECK  (no points detected)"); return False
    R, az, el = _recover(pts)
    vbest = float(pts[np.argmax(pts[:, 3]), 4])
    print(f"  recovered  R={R:5.2f} m   az={az:+6.2f}   el={el:+6.2f}   v={vbest:+5.2f} m/s")
    okB = abs(az - AZ_T) < AZ_TOL and abs(el - EL_T) < EL_TOL
    print("  RESULT:", "PASS" if okB else "CHECK  <-- if this fails, the TDM Doppler comp is missing/wrong")
    ok_all &= okB

    # pure noise should be trimmed by the confidence gate
    noise = np.random.randn(NUM_TX, NUM_RX) + 1j * np.random.randn(NUM_TX, NUM_RX)
    a, e, c = estimate_angles_search(noise)
    print(f"\n  noise snapshot -> conf={c:.1f} dB  "
          f"{'trimmed by MIN_CONF_DB' if c < MIN_CONF_DB else 'KEPT (raise MIN_CONF_DB)'}")
    return ok_all


# ============================================================
def main():
    # ---- resolve elevation mode FIRST: it rebuilds the steering dictionary, so it
    #      has to be settled before anything (incl. --selftest) touches the geometry ----
    global ELEV_MODE
    if "--noelev" in sys.argv:
        ELEV_MODE = False
    elif "--elev" in sys.argv:
        ELEV_MODE = True
    configure_geometry(ELEV_MODE)
    if ELEV_MODE:
        print(f"ELEVATION mode ON: array manifold rotated 90 deg about boresight "
              f"(ELEV_SIGN={ELEV_SIGN:+.0f}). Big ULA -> elevation (fine), lifted TX -> "
              f"azimuth (coarse). xyz stay in the world frame; no point-cloud shift.")

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
    nocomp = "--nocomp" in sys.argv
    if "--nowindow" in sys.argv:
        window_on = False
    elif "--window" in sys.argv:
        window_on = True
    else:
        window_on = WINDOW_AGG
    half_w = WINDOW_FRAMES if window_on else 0

    if src.lower().endswith(".npz"):
        # ---- replay from a saved point cache (no .bin needed) ----
        if nocomp:
            print("note: --nocomp has no effect on a cache -- the Doppler comp is applied "
                  "at build time, so it's already baked in. Reprocess the .bin to A/B it.")
        clouds, cal_label, pre_roll = load_cache(src)
        if window_on and pre_roll < WINDOW_FRAMES:
            print(f"note: temporal window +/-{WINDOW_FRAMES}f requested but the cache stored "
                  f"only {pre_roll} pre-roll frame(s) before TARGET_START -- the first "
                  f"~{WINDOW_FRAMES} output frames look back {pre_roll} frame(s) then clamp. "
                  f"Reprocess the .bin with --window (at this WINDOW_FRAMES) for a full lead-in.")
        out = str(args[1]) if len(args) > 1 else OUT
        if out == OUT:
            if window_on:
                out = out[:-4] + f"_win{WINDOW_FRAMES}" + out[-4:]
            if ELEV_MODE:
                out = out[:-4] + "_elev" + out[-4:]
        render(clouds, cal_label, out, pre_roll=pre_roll, half_w=half_w)
    else:
        # ---- process the .bin: build clouds, save the cache, then render ----
        clouds, cal_label, pre_roll = build(src, nobg=nobg, nocal=nocal, nocomp=nocomp,
                                            window=window_on)
        # distinct names so an A/B (comp vs --nocomp, window vs not, elev vs not) doesn't overwrite itself
        tag = ((".nocomp" if nocomp else "") + (f".win{WINDOW_FRAMES}" if window_on else "")
               + (".elev" if ELEV_MODE else ""))
        cache_path = os.path.splitext(src)[0] + tag + ".cache.npz"
        save_cache(clouds, cache_path, cal_label, pre_roll)
        out = OUT
        if nobg and out == "bgsub_3d.mp4":
            out = "raw_3d.mp4"
        if nocomp:
            out = out[:-4] + "_nocomp" + out[-4:]
        if window_on:
            out = out[:-4] + f"_win{WINDOW_FRAMES}" + out[-4:]
        if ELEV_MODE:
            out = out[:-4] + "_elev" + out[-4:]
        render(clouds, cal_label, out, pre_roll=pre_roll, half_w=half_w)


if __name__ == "__main__":
    main()
