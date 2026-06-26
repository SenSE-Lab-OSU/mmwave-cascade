"""
range_over_frames.py -- two diagnostic images from one capture, self-contained
(does NOT import any of the other project scripts).

    python range_over_frames.py capture_xxxx.bin [start_frame] [tx] [rx]
        start_frame  first frame to use   (default 0; skip a noisy warm-up)
        tx           TX slot 0..5          (default 0)   -- image 1 channel only
        rx           RX index 0..7         (default 0)   -- image 1 channel only

IMAGE 1  range_over_frames.png
    The old range-vs-time map for ONE virtual channel (tx, rx). For each frame:
    range-FFT the channel, average the 64 loops to denoise, keep the range
    profile, and stack every frame side by side. Truly-still scene -> HORIZONTAL
    bars (each range bin constant over time). Flicker/drift = that range was not
    stable. This is what you read stage boundaries off of. Now drawn with many
    more labelled ticks + a grid on both axes so you can read exact frame indices
    and ranges.

IMAGE 2  angle_over_frames.png
    NEW. For each frame it range-Doppler processes ALL 48 channels, picks the
    strongest few detections, and runs the manual steering-vector (Bartlett)
    angle search on each -- the same search the rest of the pipeline uses. It
    plots their azimuth (top) and elevation (bottom) versus frame, coloured by
    intensity. A static reflector at boresight sits at az~0; a moving target
    traces a track; noise-only cells are dropped (low confidence) or show up dark
    and low. These are RAW (uncalibrated) angles -- so this is also where a
    left/right mirror flip would be visible.

Reads with mmap, so a multi-minute (multi-GB) capture is fine.

----------------------------------------------------------------------------
TUNABLES -- all globals in the CONFIG block below. The only command-line inputs
are the file and the optional start/tx/rx for image 1.
  DYN_RANGE_DB    image 1 colour span below its peak (dB)
  ANG_DET_DB      image 2: keep detections within this many dB of the frame peak
  ANG_TOPK        image 2: how many strongest detections per frame to angle-search
  MIN_CONF_DB     image 2: angle-search sharpness needed to trust a point (else drop)
  ANG_MIN_BIN     image 2: ignore range bins below this (near-range coupling)
  ANG_FRAME_STEP  image 2: process every Nth frame (raise it to speed up long files)
  ANG_DB_RANGE    image 2: colour span below the brightest point (dB)
  AZ_SIGN         flip to -1.0 if targets come out mirrored left/right
  FOV_AZ/FOV_EL   angle-search half-range in az / el (deg)
  AZ_STEP/EL_STEP angle-search grid step (deg)
  tick density:   X_TICK_BINS, RANGE_TICK_STEP_M, AZ_TICK_DEG, EL_TICK_DEG
"""
import sys
import mmap
import struct
import numpy as np

# ============================================================
# CONFIG
# ============================================================

# ---- capture format (must match firmware; don't touch unless it changed) ----
SAMPLES          = 256
NUM_RX           = 8
NUM_TX           = 6                       # 6 TDMA slots
NUM_LOOPS        = 64
CHIRPS_PER_FRAME = NUM_LOOPS * NUM_TX      # 384
HEADER_MAGIC     = 0xA1B2C3D4
HEADER_BYTES     = 16
MAGIC_LE         = struct.pack("<I", HEADER_MAGIC)
DATA_PER_CHIRP   = SAMPLES * NUM_RX * 4    # 8192  (int16 I + int16 Q per sample)

# ---- range axis (MUST match current firmware -- stale values give 2x range errors) ----
C_LIGHT  = 299792458.0
ADC_RATE = 10000e3        # 10 Msps
SLOPE    = 150.06e12      # Hz/s
F0       = 76e9           # start frequency (Hz)
LAMBDA   = C_LIGHT / F0
#   range bin spacing ~3.9 cm/bin  ->  256 bins span ~10 m

# ---- image 1 (range vs time) ----
DYN_RANGE_DB = 40.0        # colour span below the peak

# ---- image 2 (angle of strongest detections vs time) ----
ANG_DET_DB     = 12.0      # keep detections within this many dB of the frame peak
ANG_TOPK       = 3         # strongest detections per frame to angle-search
MIN_CONF_DB    = 8.0       # angle-search sharpness needed to trust a point
ANG_MIN_BIN    = 4         # ignore range bins below this (near-range coupling)
ANG_FRAME_STEP = 1         # process every Nth frame (raise to speed up long files)
ANG_DB_RANGE   = 30.0      # colour span below the brightest point

# ---- angle search field-of-view / resolution ----
AZ_SIGN  = +1.0            # flip to -1.0 if targets come out mirrored left/right
FOV_AZ   = 60.0
FOV_EL   = 30.0
AZ_STEP  = 0.5
EL_STEP  = 1.0

# ---- tick density (how many numbers on the axes) ----
X_TICK_BINS      = 20      # up to this many labelled frame ticks
RANGE_TICK_STEP_M = 0.5    # a labelled range tick every this many metres
AZ_TICK_DEG      = 10      # azimuth tick spacing (deg)
EL_TICK_DEG      = 10      # elevation tick spacing (deg)

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
# FILE I/O  (mmap; header sits AFTER each chirp's data)
# ============================================================
def chirp_offsets(mm):
    starts = []
    i = mm.find(MAGIC_LE)
    while i != -1:
        if i >= DATA_PER_CHIRP:
            starts.append(i - DATA_PER_CHIRP)
        i = mm.find(MAGIC_LE, i + HEADER_BYTES)
    return starts


def channel_profile(mm, tx_offsets, rx, win):
    """64 loop chirps for one TX -> one [SAMPLES] range profile for (tx, rx)."""
    loops = np.empty((len(tx_offsets), SAMPLES), complex)
    for k, off in enumerate(tx_offsets):
        buf = np.frombuffer(mm, np.uint8, DATA_PER_CHIRP, off)
        iq  = buf.view(np.int16).astype(np.float32)
        c   = iq[0::2] + 1j * iq[1::2]               # 2048 complex
        c   = c.reshape(NUM_RX, SAMPLES)             # [rx, sample]
        loops[k] = c[rx]
    rng = np.fft.fft(loops * win[None, :], axis=1)   # range FFT per loop
    return np.abs(rng).mean(axis=0)                  # denoise across the 64 loops


def frame_cube(mm, starts, f):
    """Frame f -> complex cube [loop, tx, rx, sample]."""
    sel = starts[f * CHIRPS_PER_FRAME:(f + 1) * CHIRPS_PER_FRAME]
    blk = np.empty((CHIRPS_PER_FRAME, DATA_PER_CHIRP), np.uint8)
    for k, off in enumerate(sel):
        blk[k] = np.frombuffer(mm, np.uint8, DATA_PER_CHIRP, off)
    iq = blk.view(np.int16).astype(np.float32)
    c = iq[:, 0::2] + 1j * iq[:, 1::2]
    c = c.reshape(CHIRPS_PER_FRAME, NUM_RX, SAMPLES)
    return c.reshape(NUM_LOOPS, NUM_TX, NUM_RX, SAMPLES)


# ============================================================
# RANGE FFT -> Doppler FFT + detection
# ============================================================
def range_fft(cube):
    win = np.hanning(SAMPLES).astype(np.float32)
    return np.fft.fft(cube * win[None, None, None, :], axis=3)


def range_doppler(rng):
    win = np.hanning(NUM_LOOPS).astype(np.float32)
    dop = np.fft.fft(rng * win[:, None, None, None], axis=0)
    dop = np.fft.fftshift(dop, axes=0)
    mag = np.abs(dop).sum(axis=(1, 2))               # [doppler, range]
    return dop, mag


def detect(mag, db_below_peak, min_range_bin, max_targets):
    m = mag.copy()
    m[:, :min_range_bin] = 0.0
    thr = m.max() * 10 ** (-db_below_peak / 20.0)
    cells = np.argwhere(m > thr)
    if len(cells) == 0:
        return cells
    strength = m[cells[:, 0], cells[:, 1]]
    order = np.argsort(strength)[::-1][:max_targets]
    return cells[order]


# ============================================================
# MANUAL ANGLE SEARCH  (steering-vector / Bartlett over real element geometry)
# ============================================================
_xs, _zs = [], []
for _tx in range(NUM_TX):
    for _rx in range(NUM_RX):
        _xs.append((TX_POS[_tx, 0] + RX_X[_rx]) * 0.5)   # half-lambda cols -> lambda
        _zs.append(TX_POS[_tx, 1])
_xs = np.asarray(_xs)
_zs = np.asarray(_zs)

_az = np.arange(-FOV_AZ, FOV_AZ + AZ_STEP, AZ_STEP)
_el = np.arange(-FOV_EL, FOV_EL + EL_STEP, EL_STEP)
_AZ, _EL = np.meshgrid(np.radians(_az), np.radians(_el), indexing="ij")
_u = AZ_SIGN * np.sin(_AZ) * np.cos(_EL)
_w = np.sin(_EL)
_phase = 2 * np.pi * (_u[..., None] * _xs[None, None, :] +
                      _w[..., None] * _zs[None, None, :])
_STEER_C = np.conj(np.exp(1j * _phase)).reshape(-1, _xs.size)


def estimate_angles_search(snap):
    """snap [NUM_TX, NUM_RX] complex -> (az_deg, el_deg, conf_db); NaN if untrusted."""
    s = snap.reshape(-1)
    P = np.abs(_STEER_C @ s) ** 2
    k = int(np.argmax(P))
    conf_db = 10.0 * np.log10(P[k] / (np.median(P) + 1e-30))
    if conf_db < MIN_CONF_DB:
        return np.nan, np.nan, conf_db
    iaz, iel = np.unravel_index(k, (_az.size, _el.size))
    return float(_az[iaz]), float(_el[iel]), float(conf_db)


# ============================================================
# IMAGE 1 DATA: range-vs-time matrix for one channel
# ============================================================
def range_time_matrix(mm, starts, start, n_frames, tx, rx):
    win = np.hanning(SAMPLES).astype(np.float32)
    n_use = n_frames - start
    M = np.empty((n_use, SAMPLES))
    for j in range(n_use):
        f0 = (start + j) * CHIRPS_PER_FRAME
        frame_offsets = starts[f0:f0 + CHIRPS_PER_FRAME]
        tx_offsets = frame_offsets[tx::NUM_TX]       # this TX's 64 loop chirps
        M[j] = channel_profile(mm, tx_offsets, rx, win)
    return M


# ============================================================
# IMAGE 2 DATA: angle of strongest detections per frame
# ============================================================
def angle_track(mm, starts, start, n_frames):
    fr_list, az_list, el_list, db_list = [], [], [], []
    for fr in range(start, n_frames, ANG_FRAME_STEP):
        rng = range_fft(frame_cube(mm, starts, fr))
        dop, mag = range_doppler(rng)
        cells = detect(mag, ANG_DET_DB, ANG_MIN_BIN, ANG_TOPK)
        for dbin, rbin in cells:
            snap = dop[dbin, :, :, rbin]
            az, el, conf = estimate_angles_search(snap)
            if not np.isfinite(az):
                continue
            fr_list.append(fr)
            az_list.append(az)
            el_list.append(el)
            db_list.append(20.0 * np.log10(float(mag[dbin, rbin]) + 1e-6))
        if fr % 50 == 0:
            print(f"\r  angle track: frame {fr}/{n_frames}", end="", flush=True)
    print()
    return (np.array(fr_list), np.array(az_list),
            np.array(el_list), np.array(db_list))


# ============================================================
# PLOTS
# ============================================================
def plot_range_time(M, start, n_frames, tx, rx):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator, MultipleLocator, AutoMinorLocator

    img = 20.0 * np.log10(M.T + 1e-6)                # [range, frame]
    peak = img.max()
    r_max = range_bin_to_m(SAMPLES - 1)

    fig, ax = plt.subplots(figsize=(12, 6))
    im = ax.imshow(img, aspect="auto", origin="lower", cmap="viridis",
                   extent=[start, n_frames, 0.0, r_max],
                   vmin=peak - DYN_RANGE_DB, vmax=peak)
    ax.set_xlabel("frame index  (time ->)")
    ax.set_ylabel("range (m)")
    ax.set_title(f"range over frames  --  TX{tx} RX{rx}  (horizontal bars = stable)")

    ax.xaxis.set_major_locator(MaxNLocator(nbins=X_TICK_BINS, integer=True))
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_major_locator(MultipleLocator(RANGE_TICK_STEP_M))
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.tick_params(axis="both", labelsize=8)
    ax.tick_params(axis="x", labelrotation=90)
    ax.grid(which="major", color="w", alpha=0.25, linewidth=0.5)
    ax.grid(which="minor", color="w", alpha=0.10, linewidth=0.4)

    fig.colorbar(im, ax=ax, label="magnitude (dB)")
    fig.tight_layout()
    fig.savefig("range_over_frames.png", dpi=120)
    plt.close(fig)
    print("saved range_over_frames.png")


def plot_angle_track(frames, az, el, db, start, n_frames):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator, MultipleLocator, AutoMinorLocator

    if len(frames):
        vmax = float(db.max()); vmin = vmax - ANG_DB_RANGE
    else:
        vmax, vmin = 0.0, -ANG_DB_RANGE

    fig, (ax_az, ax_el) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    sc = ax_az.scatter(frames, az, c=db, cmap="viridis", vmin=vmin, vmax=vmax,
                       s=16, edgecolors="none")
    ax_el.scatter(frames, el, c=db, cmap="viridis", vmin=vmin, vmax=vmax,
                  s=16, edgecolors="none")

    ax_az.set_ylabel("azimuth (deg)")
    ax_az.set_ylim(-FOV_AZ, FOV_AZ)
    ax_az.axhline(0.0, color="k", linewidth=0.6, alpha=0.4)   # boresight
    ax_az.set_title("angle of strongest detections over frames  "
                    "(colour = intensity; raw / uncalibrated angles)")

    ax_el.set_ylabel("elevation (deg)")
    ax_el.set_ylim(-FOV_EL, FOV_EL)
    ax_el.axhline(0.0, color="k", linewidth=0.6, alpha=0.4)
    ax_el.set_xlabel("frame index  (time ->)")

    for ax, step in ((ax_az, AZ_TICK_DEG), (ax_el, EL_TICK_DEG)):
        ax.set_xlim(start, n_frames)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=X_TICK_BINS, integer=True))
        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_major_locator(MultipleLocator(step))
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.tick_params(axis="both", labelsize=8)
        ax.tick_params(axis="x", labelrotation=90)
        ax.grid(which="major", alpha=0.25, linewidth=0.5)
        ax.grid(which="minor", alpha=0.10, linewidth=0.4)

    fig.colorbar(sc, ax=[ax_az, ax_el], label="intensity (dB)")
    fig.savefig("angle_over_frames.png", dpi=120)
    plt.close(fig)
    print("saved angle_over_frames.png")


# ============================================================
def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    path  = sys.argv[1]
    start = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    tx    = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    rx    = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    f  = open(path, "rb")
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    starts = chirp_offsets(mm)
    n_frames = len(starts) // CHIRPS_PER_FRAME
    if start >= n_frames:
        raise SystemExit(f"file has only {n_frames} full frames; start={start} too large")

    print(f"frames in file:   {n_frames}")
    print(f"frames analyzed:  {n_frames - start}  (from frame {start})")
    print(f"image 1 channel:  TX{tx} RX{rx}")

    # ---- image 1 ----
    M = range_time_matrix(mm, starts, start, n_frames, tx, rx)
    plot_range_time(M, start, n_frames, tx, rx)

    # ---- image 2 ----
    frames, az, el, db = angle_track(mm, starts, start, n_frames)
    print(f"angle points kept: {len(frames)} "
          f"(top {ANG_TOPK}/frame, every {ANG_FRAME_STEP} frame(s), "
          f"conf >= {MIN_CONF_DB:.0f} dB)")
    plot_angle_track(frames, az, el, db, start, n_frames)

    mm.close(); f.close()


if __name__ == "__main__":
    main()
