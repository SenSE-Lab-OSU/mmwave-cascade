"""
bgsub_3d.py -- ONE self-contained file: background-subtracted 3D point-cloud
movie of a moving target, using the manual steering-vector (Bartlett) angle
search, with per-channel phase calibration computed INLINE from the corner-
reflector stage of the same capture (no separate make_calibration step, no
reference_48ch.npy file needed).

    python bgsub_3d.py capture.bin              # calibration ON  (uses CAL_FRAMES below)
    python bgsub_3d.py capture.bin --nocal      # calibration OFF (A/B the effect)
    python bgsub_3d.py capture.bin --nobg       # no background suppression (raw)
    python bgsub_3d.py --selftest               # check the math, no file needed

There are NO value flags to type on the command line. Everything you'd want to
tune lives in the CONFIG block below as a plain global -- edit it, then run.
The only command-line switches are the on/off toggles: --nocal, --nobg, --selftest.

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

Detection / trust thresholds (all in dB):
  DET_DB        a (range,Doppler) cell is kept as a point only if it's within
                this many dB of the strongest cell IN THAT FRAME. Lower keeps
                more (weaker) points; higher keeps only the brightest. On
                background-subtracted data the TARGET is the reference peak, so
                12-20 dB is right (the instructor's 40 dB was for raw data with
                wall clutter as the reference -- different baseline).
  FLOOR_DB      a whole frame is skipped unless its peak rises this many dB above
                the leftover background residual. This is the "is there even a
                target this frame" gate. Higher = stricter (fewer noise-only
                frames get drawn).
  MIN_CONF_DB   peak-to-median sharpness the angle search must reach to trust an
                angle. Below it, the point is DROPPED instead of being plotted at
                a random edge angle. This is what keeps noise cells out of the
                cloud. Raise it if you see junk points; lower it if real weak
                targets vanish.
  GATE_DROP_DB  --nobg mode only: in raw mode (no background) a frame is kept if
                its peak is within this many dB of the loudest frame overall.

Angle search field-of-view / resolution:
  AZ_SIGN       +1.0 normally; flip to -1.0 if real targets come out mirrored
                left/right. (With good calibration the mirror flip should be gone,
                so this should stay +1.0 -- it's the manual fallback.)
  FOV_AZ/FOV_EL believable search half-range in az / el (deg).
  AZ_STEP/EL_STEP  grid resolution (deg). Smaller = finer but slower to precompute.

3D scene limits (m) and view:
  XLIM   left-right half-width of the plot (x in -XLIM..+XLIM)
  YMAX   forward range shown (y in 0..YMAX)
  ZLIM   up-down half-width (z in -ZLIM..+ZLIM). Elevation is the weakest
         measurement, so keep this modest and read z loosely.
  VIEW_ELEV / VIEW_AZIM  camera angle for the 3D axes.
  DB_RANGE  color dynamic range (top point = brightest; DB_RANGE below = darkest).

Output:
  FPS    playback frame rate of the movie.
  OUT    output filename. .mp4 if ffmpeg is available, else it falls back to .gif.
"""

import sys
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

# ---- frame ranges (read off your range_over_frames.png stage map) ----
BG_FRAMES    = (1000, 1800)   # empty-scene frames -> background template
TARGET_START = 1800           # first frame to build the cloud from (moving-target stage)
CAL_FRAMES   = (0, 750)     # reflector-at-boresight frames; set None to disable cal
CAL_RANGE_M  = 4.2            # where the reflector sat (m); None = auto-pick strongest

# ---- detection / trust thresholds (dB) ----
DET_DB       = 12.0           # keep cells within this many dB of the frame's peak
FLOOR_DB     = 6.0            # skip a frame unless its peak is this far over residual
MIN_CONF_DB  = 8.0            # angle-search sharpness needed to trust (else drop point)
GATE_DROP_DB = 30.0           # --nobg only: keep frames within this dB of loudest frame

# ---- angle search field-of-view / resolution ----
AZ_SIGN  = +1.0               # flip to -1.0 if targets come out mirrored left/right
FOV_AZ   = 60.0               # azimuth   search half-range (deg)
FOV_EL   = 30.0               # elevation search half-range (deg)
AZ_STEP  = 0.5                # azimuth grid step (deg)
EL_STEP  = 1.0                # elevation grid step (deg)

# ---- scene limits (m) + view ----
XLIM     = 6.0                # x in -XLIM .. +XLIM   (left-right)
YMAX     = 8.0               # y in 0 .. YMAX        (forward)
ZLIM     = 3                # z in -ZLIM .. +ZLIM   (up-down; weakest axis)
# left panel = 3D side view: elev=0, azim=0 looks straight down the x-axis, so x
# (left-right) is collapsed into the screen and you read forward (y) vs height (z).
# Nudge VIEW_ELEV up a few deg if the floor looks too edge-on.
VIEW_ELEV = 0.0
VIEW_AZIM = 45.0
DB_RANGE  = 30.0              # color dynamic range (dB below the brightest point)

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
    az/el are NaN when the winning peak isn't sharp enough to trust (noise)."""
    s = snap.reshape(-1)                             # (48,)
    P = np.abs(_STEER_C @ s) ** 2                    # match score over the grid
    k = int(np.argmax(P))
    conf_db = 10.0 * np.log10(P[k] / (np.median(P) + 1e-30))
    if conf_db < MIN_CONF_DB:
        return np.nan, np.nan, conf_db
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


# ============================================================
# ONE FRAME -> (x, y, z, amp) points
# ============================================================
def cloud(clean, cal, det_db, gate):
    """clean cube -> (x, y, z, amp) points. Empty if the frame doesn't clear the gate."""
    rng = range_fft(clean)
    dop, mag = range_doppler(rng)
    if mag.max() < gate:
        return np.zeros((0, 4))
    pts = []
    for dbin, rbin in detect(mag, det_db):
        snap = dop[dbin, :, :, rbin]                 # [tx, rx]
        if cal is not None:
            snap = snap * cal                        # remove per-channel phase offsets
        az, el, conf = estimate_angles_search(snap)
        if not np.isfinite(az):                      # peak not sharp enough -> drop
            continue
        x, y, z = to_xyz(range_bin_to_m(rbin), az, el)
        pts.append((x, y, z, float(mag[dbin, rbin])))
    return np.array(pts) if pts else np.zeros((0, 4))


# ============================================================
# BUILD all clouds over the target frames
# ============================================================
def build(path, nobg, nocal):
    f = open(path, "rb")
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    starts = chirp_offsets(mm)
    n_frames = len(starts) // CHIRPS_PER_FRAME
    print(f"frames in file: {n_frames}")

    # ---- calibration (inline) ----
    cal = None
    if nocal or CAL_FRAMES is None:
        print("calibration OFF" + (" (--nocal)" if nocal else " (CAL_FRAMES=None)"))
    else:
        cal = build_calibration(mm, starts, CAL_FRAMES, CAL_RANGE_M, n_frames)
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

    # ---- per-frame clouds ----
    clouds = []
    for fr in range(target, n_frames):
        clouds.append(cloud(frame_cube(mm, starts, fr) - template[None], cal, DET_DB, gate))
        if fr % 50 == 0:
            print(f"\r  clouds: frame {fr}/{n_frames}", end="", flush=True)
    print()
    mm.close(); f.close()
    print(f"  {sum(1 for p in clouds if len(p))}/{len(clouds)} frames have points")
    return clouds, cal_label


# ============================================================
# RENDER the movie
# ============================================================
def render(clouds, cal_label, out):
    """Side-by-side, synchronized: LEFT = 3D side view (x obscured -> forward y vs
    height z), RIGHT = top-down (x left-right vs y forward, z dropped). Both panels
    are driven by the same frame index, so they always show the same instant."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

    seen = [p for p in clouds if len(p)]
    allpts = np.vstack(seen) if seen else np.zeros((1, 4))
    vmax = float((20 * np.log10(allpts[:, 3] + 1e-6)).max()); vmin = vmax - DB_RANGE

    fig = plt.figure(figsize=(15, 7))

    # ---- LEFT: 3D side view ----
    ax3 = fig.add_subplot(121, projection="3d")
    scat3 = ax3.scatter([0], [0], [0], c=[vmin], cmap="viridis",
                        vmin=vmin, vmax=vmax, s=20, depthshade=False)
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
    ax3.set_title(f"side view ({cal_label})")

    # ---- RIGHT: top-down (bird's eye) ----
    ax2 = fig.add_subplot(122)
    scat2 = ax2.scatter([0], [0], c=[vmin], cmap="viridis",
                        vmin=vmin, vmax=vmax, s=20)
    ax2.set_xlim(-XLIM, XLIM); ax2.set_ylim(0, YMAX)
    ax2.set_xlabel("x  left-right (m)")
    ax2.set_ylabel("y  forward (m)")
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(alpha=0.25)
    ax2.set_title("top down")

    fig.colorbar(scat2, ax=[ax3, ax2], label="intensity (dB)", shrink=0.6)
    sup = fig.suptitle("")

    def update(i):
        p = clouds[i]
        if len(p):
            db = 20 * np.log10(p[:, 3] + 1e-6)
            scat3._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
            scat3.set_array(db)
            scat2.set_offsets(np.column_stack((p[:, 0], p[:, 1])))
            scat2.set_array(db)
        else:
            scat3._offsets3d = ([], [], [])
            scat3.set_array(np.array([]))
            scat2.set_offsets(np.empty((0, 2)))
            scat2.set_array(np.array([]))
        sup.set_text(f"frame {i + 1}/{len(clouds)}   {len(p)} pts   ({cal_label})")
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
    pts = cloud(cube, cal=None, det_db=DET_DB, gate=0.0)
    if len(pts) == 0:
        print("  RESULT: CHECK  (no points detected)")
        return False
    best = pts[np.argmax(pts[:, 3])]
    x, y, z, _ = best
    R = float(np.sqrt(x * x + y * y + z * z))
    az = float(np.degrees(np.arctan2(x, y)))
    el = float(np.degrees(np.arcsin(np.clip(z / max(R, 1e-9), -1, 1))))
    print(f"  recovered  R={R:5.2f} m   az={az:+6.2f} deg   el={el:+6.2f} deg")
    print(f"  xyz        x={x:+5.2f}  y={y:+5.2f}  z={z:+5.2f}  (m)")
    ok = abs(R - 6.0) < 0.15 and abs(az - 20) < 2.0 and abs(el - 8) < 3.0
    print("  RESULT:", "PASS" if ok else "CHECK")

    # pure noise should be rejected by the angle search
    noise = np.random.randn(NUM_TX, NUM_RX) + 1j * np.random.randn(NUM_TX, NUM_RX)
    a, e, c = estimate_angles_search(noise)
    print(f"  noise snapshot -> conf={c:.1f} dB  "
          f"{'dropped (good)' if not np.isfinite(a) else 'KEPT (bad)'}")
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
    nobg = "--nobg" in sys.argv
    nocal = "--nocal" in sys.argv
    clouds, cal_label = build(args[0], nobg=nobg, nocal=nocal)
    out = OUT
    if nobg and out == "bgsub_3d.mp4":
        out = "raw_3d.mp4"
    render(clouds, cal_label, out)


if __name__ == "__main__":
    main()