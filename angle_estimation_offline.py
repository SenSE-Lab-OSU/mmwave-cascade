"""
angle_estimation_offline.py

Reads ONE frame from a captured .bin, finds targets, estimates the azimuth and
elevation angle of each one, and turns (range, azimuth, elevation) into a 3D
point cloud (x, y, z).

This is an OFFLINE / read-only tool. It never touches the live capture path.

Usage:
    python angle_estimation_offline.py  capture_xxxx.bin   [frame_index]
    python angle_estimation_offline.py  --selftest          # no file needed

The --selftest mode builds a fake frame with a target at a KNOWN angle, runs the
whole pipeline, and checks that it recovers that angle. Run it first to confirm
the math before trusting it on real data.
"""

import sys
import struct
import numpy as np

# ============================================================
# CONFIG  (matches the firmware + capture_and_display_v2.py)
# ============================================================
SAMPLES          = 256
NUM_RX           = 8
NUM_TX           = 6          # 6 TDMA slots
NUM_LOOPS        = 64
CHIRPS_PER_FRAME = NUM_LOOPS * NUM_TX          # 384

HEADER_MAGIC   = 0xA1B2C3D4
HEADER_BYTES   = 16
MAGIC_LE       = struct.pack("<I", HEADER_MAGIC)
DATA_PER_CHIRP = SAMPLES * NUM_RX * 4          # 8192  (int16 I + int16 Q)
BLOCK_BYTES    = DATA_PER_CHIRP + HEADER_BYTES # 8208

# ---- range axis calibration (from the chirp profile) ----
C_LIGHT  = 299792458.0
ADC_RATE = 10000e3      # 10 Msps
SLOPE    = 75.03e12     # 75.03 MHz/us in Hz/s
F0       = 77e9         # start freq
LAMBDA   = C_LIGHT / F0

def range_bin_to_m(bin_idx):
    dR = C_LIGHT * ADC_RATE / (2.0 * SLOPE * SAMPLES)   # ~7.8 cm/bin
    return bin_idx * dR

# ============================================================
# ANTENNA GEOMETRY   (half-lambda units for x, lambda units for z)
#   - derived from Fig 4-1 and the firmware firing order in common.c
#   - tx slot order: 0-2 = Dev1 TX0/1/2, 3-5 = Dev2 TX0/1/2
#   - slot 2 is the lifted (elevation) transmitter
# ============================================================
TX_POS = np.array([
    [0,  0.0],    # slot 0  Dev1.TX0
    [4,  0.0],    # slot 1  Dev1.TX1
    [2,  0.8],    # slot 2  Dev1.TX2   <-- lifted 0.8 lambda (elevation)
    [7,  0.0],    # slot 3  Dev2.TX0
    [11, 0.0],    # slot 4  Dev2.TX1
    [15, 0.0],    # slot 5  Dev2.TX2
], dtype=float)
RX_X = np.array([0, 1, 2, 3, 19, 20, 21, 22], dtype=float)   # half-lambda

AZ_COLS = 38            # contiguous half-lambda columns, 0..37  (spans 18.5 lambda)
ELEV_DZ = 0.8           # vertical separation of the two rows, in lambda
# ---- calibration (optional; created by save_reference.py) ----
try:
    _REF = np.load("reference_48ch.npy")
    CAL = np.conj(_REF) / np.abs(_REF)        # phase calibration, shape [tx, rx]
except FileNotFoundError:
    CAL = None

# ============================================================
# 1. READ ONE FRAME FROM THE .bin
#    Same block format and IQ handling as the capture script.
# ============================================================
def read_frame(path, frame_index=5):
    raw = open(path, "rb").read()

    # Each chirp block is [8192 bytes data][16 byte header]; the magic word
    # marks the header, which sits AFTER the data. Find every header, then the
    # data for that chirp is the 8192 bytes just before it.
    starts = []
    i = raw.find(MAGIC_LE)
    while i != -1:
        if i >= DATA_PER_CHIRP:
            starts.append(i - DATA_PER_CHIRP)
        i = raw.find(MAGIC_LE, i + HEADER_BYTES)

    # Chirps are contiguous and in order, so frame N is just the Nth block of 384.
    lo = frame_index * CHIRPS_PER_FRAME
    sel = starts[lo: lo + CHIRPS_PER_FRAME]
    if len(sel) < CHIRPS_PER_FRAME:
        raise SystemExit(f"file only has {len(starts)//CHIRPS_PER_FRAME} full frames")

    blocks = np.empty((CHIRPS_PER_FRAME, DATA_PER_CHIRP), np.uint8)
    for k, off in enumerate(sel):
        blocks[k] = np.frombuffer(raw, np.uint8, count=DATA_PER_CHIRP, offset=off)

    iq = blocks.view(np.int16).astype(np.float32)          # (384, 4096)
    c  = iq[:, 0::2] + 1j * iq[:, 1::2]                     # (384, 2048) complex
    c  = c.reshape(CHIRPS_PER_FRAME, NUM_RX, SAMPLES)       # [chirp, rx, sample]
    c  = c.reshape(NUM_LOOPS, NUM_TX, NUM_RX, SAMPLES)      # chirp = loop*6 + tx
    return c                                                # [loop, tx, rx, sample]


# ============================================================
# 2. RANGE FFT  -> how far
# ============================================================
def range_fft(cube):
    win = np.hanning(SAMPLES).astype(np.float32)
    return np.fft.fft(cube * win[None, None, None, :], axis=3)   # [loop, tx, rx, range]


# ============================================================
# 3. DOPPLER FFT + DETECTION -> which (range, velocity) cells hold a target
# ============================================================
def range_doppler(rng):
    win = np.hanning(NUM_LOOPS).astype(np.float32)
    dop = np.fft.fft(rng * win[:, None, None, None], axis=0)
    dop = np.fft.fftshift(dop, axes=0)               # [doppler, tx, rx, range]
    mag = np.abs(dop).sum(axis=(1, 2))               # [doppler, range], summed over channels
    return dop, mag

def detect(mag, db_below_peak=18.0, min_range_bin=4, max_targets=300):
    m = mag.copy()
    m[:, :min_range_bin] = 0.0                       # kill near-range coupling
    thr = m.max() * 10 ** (-db_below_peak / 20.0)
    cells = np.argwhere(m > thr)                     # rows of [doppler_bin, range_bin]
    strength = m[cells[:, 0], cells[:, 1]]
    order = np.argsort(strength)[::-1][:max_targets]
    return cells[order]


# ============================================================
# 4. ANGLE ESTIMATION
# ============================================================
def snapshot_to_array(snap):
    """
    snap: 48 complex values, shape [tx, rx].
    Place them on the 2D virtual grid A[row, col]:
      row 0 = main horizontal line (z = 0)
      row 1 = lifted line          (z = 0.8 lambda)
      col   = horizontal position in half-lambda (0..37)
    Overlapping taps (same row+col) are averaged.
    """
    A = np.zeros((2, AZ_COLS), dtype=complex)
    n = np.zeros((2, AZ_COLS))
    for tx in range(NUM_TX):
        x_tx, z_tx = TX_POS[tx]
        row = 0 if z_tx == 0 else 1
        for rx in range(NUM_RX):
            col = int(x_tx + RX_X[rx])
            A[row, col] += snap[tx, rx]
            n[row, col] += 1
    n[n == 0] = 1
    return A / n

def _peak_freq(x, nfft=512):
    """FFT a 1D array, return the peak's spatial frequency in cycles/element."""
    w = np.hanning(len(x))
    X = np.fft.fftshift(np.fft.fft(x * w, nfft))
    p = np.abs(X)
    k = int(np.argmax(p))
    # parabolic interpolation for a sub-bin peak (finer angle)
    if 0 < k < nfft - 1:
        a, b, c = p[k - 1], p[k], p[k + 1]
        k = k + 0.5 * (a - c) / (a - 2 * b + c + 1e-12)
    return (k - nfft / 2) / nfft        # cycles per element, in [-0.5, 0.5)

def estimate_angles(A):
    """Return (azimuth_deg, elevation_deg) for one target snapshot."""
    # --- elevation: up/down phase difference on the 8 paired columns ---
    cols = np.where((np.abs(A[0]) > 0) & (np.abs(A[1]) > 0))[0]
    prod = np.mean(A[1, cols] * np.conj(A[0, cols]))   # average the complex products
    dphi = np.angle(prod)                              # = 2*pi * 0.8 * sin(elev)
    sin_el = np.clip(dphi / (2 * np.pi * ELEV_DZ), -1, 1)
    elev = np.arcsin(sin_el)

    # --- azimuth: phase ramp across the 38-element main row ---
    f = _peak_freq(A[0])                               # cycles per element
    u = 2.0 * f                                        # = sin(az) * cos(el),  since spacing = 0.5 lambda
    sin_az = np.clip(u / max(np.cos(elev), 1e-3), -1, 1)
    az = -np.arcsin(sin_az)
    return np.degrees(az), np.degrees(elev)


def to_xyz(range_m, az_deg, el_deg):
    az, el = np.radians(az_deg), np.radians(el_deg)
    x = range_m * np.cos(el) * np.sin(az)     # left-right
    y = range_m * np.cos(el) * np.cos(az)     # forward (boresight)
    z = range_m * np.sin(el)                  # up-down
    return x, y, z


# ============================================================
# FULL PIPELINE on a real frame
# ============================================================
def build_point_cloud(cube):
    rng      = range_fft(cube)
    dop, mag = range_doppler(rng)
    cells    = detect(mag)

    pts = []
    for dbin, rbin in cells:
        snap = dop[dbin, :, :, rbin]                 # [tx, rx] = 48 complex
        if CAL is not None:
            snap = snap * CAL                        # remove per-channel mismatch
        A = snapshot_to_array(snap)
        az, el = estimate_angles(A)
        R = range_bin_to_m(rbin)
        x, y, z = to_xyz(R, az, el)
        amp = float(mag[dbin, rbin])
        pts.append((x, y, z, R, az, el, amp))
    return np.array(pts)


# ============================================================
# SELF TEST: synthesise a known target and check we recover it
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
            x_lam = (x_tx + RX_X[rx]) * 0.5          # element x in lambda
            z_lam = z_tx                             # element z in lambda
            phase = 2 * np.pi * (x_lam * np.sin(az) * np.cos(el) + z_lam * np.sin(el))
            cube[:, tx, rx, :] = range_sig[None, :] * np.exp(1j * phase)
    noise = (np.random.randn(*cube.shape) + 1j * np.random.randn(*cube.shape))
    cube += noise * 10 ** (-snr_db / 20)
    return cube

def selftest():
    print("self-test: target at R=6.0 m, az=+20 deg, el=+8 deg")
    cube = _synth_cube(6.0, 20.0, 8.0)
    pts = build_point_cloud(cube)
    # strongest detection
    best = pts[np.argmax(pts[:, 6])]
    x, y, z, R, az, el, _ = best
    print(f"  recovered  R={R:5.2f} m   az={az:+6.2f} deg   el={el:+6.2f} deg")
    print(f"  xyz        x={x:+5.2f}  y={y:+5.2f}  z={z:+5.2f}  (m)")
    ok = abs(R - 6.0) < 0.15 and abs(az - 20) < 2.0 and abs(el - 8) < 2.0
    print("  RESULT:", "PASS" if ok else "CHECK")
    return ok


# ============================================================
# PLOT / SAVE the cloud
# ============================================================
def save_cloud(pts, out_png="point_cloud.png"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    s = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                   c=20 * np.log10(pts[:, 6] + 1e-6), cmap="viridis", s=12)
    ax.set_xlabel("x  left-right (m)")
    ax.set_ylabel("y  forward (m)")
    ax.set_zlabel("z  up-down (m)")
    fig.colorbar(s, label="intensity (dB)")
    ax.set_title(f"point cloud  ({len(pts)} points)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    print(f"saved {out_png}")


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--selftest":
        selftest()
        return
    if len(sys.argv) < 2:
        print(__doc__)
        return
    path = sys.argv[1]
    frame_index = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    cube = read_frame(path, frame_index)
    pts = build_point_cloud(cube)
    order = np.argsort(pts[:, 6])[::-1]
    print("  R(m)   az     el    dB")
    for i in order[:len(pts)]:
        x, y, z, R, az, el, amp = pts[i]
        print(f" {R:5.2f}  {az:+5.1f}  {el:+5.1f}  {20*np.log10(amp):5.1f}")
    print(f"{len(pts)} points")
    np.save("point_cloud.npy", pts)
    save_cloud(pts)

if __name__ == "__main__":
    main()
