"""
movie_topdown.py -- top-down (bird's-eye) point-cloud movie, with and without calibration.

Renders TWO movies from one capture, in a single pass:
  topdown_nocal.<ext>   uncalibrated
  topdown_cal.<ext>     calibrated with reference_48ch.npy   (skipped if that file is absent)

Looking straight down: x = left-right (-2..2 m), y = forward (0..4 m), radar at the origin.

    python movie_topdown.py capture_xxxx.bin
    python movie_topdown.py capture_xxxx.bin --fps 20 --db 12 --ext mp4
"""
import sys
import numpy as np
import angle_estimation_offline as ae

XLIM = (-2.0, 2.0)
YLIM = (0.0, 4.0)


def frames(path):
    raw = open(path, "rb").read()
    starts = []
    i = raw.find(ae.MAGIC_LE)
    while i != -1:
        if i >= ae.DATA_PER_CHIRP:
            starts.append(i - ae.DATA_PER_CHIRP)
        i = raw.find(ae.MAGIC_LE, i + ae.HEADER_BYTES)
    for f in range(len(starts) // ae.CHIRPS_PER_FRAME):
        sel = starts[f * ae.CHIRPS_PER_FRAME:(f + 1) * ae.CHIRPS_PER_FRAME]
        blk = np.empty((ae.CHIRPS_PER_FRAME, ae.DATA_PER_CHIRP), np.uint8)
        for k, off in enumerate(sel):
            blk[k] = np.frombuffer(raw, np.uint8, ae.DATA_PER_CHIRP, off)
        iq = blk.view(np.int16).astype(np.float32)
        c = iq[:, 0::2] + 1j * iq[:, 1::2]
        c = c.reshape(ae.CHIRPS_PER_FRAME, ae.NUM_RX, ae.SAMPLES)
        yield c.reshape(ae.NUM_LOOPS, ae.NUM_TX, ae.NUM_RX, ae.SAMPLES)


def clouds_both(cube, cal, db):
    """One frame -> (nocal_points, cal_points), each an array of (x, y, amp).
    The range/Doppler FFTs are done once; only the per-channel angle step differs."""
    rng = ae.range_fft(cube)
    dop, mag = ae.range_doppler(rng)
    cells = ae.detect(mag, db_below_peak=db)
    nocal, withcal = [], []
    for dbin, rbin in cells:
        snap = dop[dbin, :, :, rbin]
        R = ae.range_bin_to_m(rbin)
        amp = float(mag[dbin, rbin])
        az, el = ae.estimate_angles(ae.snapshot_to_array(snap))
        x, y, _ = ae.to_xyz(R, az, el)
        nocal.append((x, y, amp))
        if cal is not None:
            az2, el2 = ae.estimate_angles(ae.snapshot_to_array(snap * cal))
            x2, y2, _ = ae.to_xyz(R, az2, el2)
            withcal.append((x2, y2, amp))
    pack = lambda L: np.array(L) if L else np.zeros((0, 3))
    return pack(nocal), pack(withcal)


def render(clouds, out, fps, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

    allpts = np.vstack([p for p in clouds if len(p)]) if any(len(p) for p in clouds) \
        else np.zeros((1, 3))
    dbvals = 20 * np.log10(allpts[:, 2] + 1e-6)
    vmax = float(dbvals.max()); vmin = vmax - 30

    fig, ax = plt.subplots(figsize=(6, 6))
    scat = ax.scatter([0], [0], c=[vmin], cmap="viridis", vmin=vmin, vmax=vmax, s=22)
    ax.plot(0, 0, marker="^", color="k", ms=11)            # radar at origin
    ax.set_xlim(*XLIM); ax.set_ylim(*YLIM); ax.set_aspect("equal")
    ax.set_xlabel("x  left-right (m)")
    ax.set_ylabel("y  forward (m)")
    ax.grid(alpha=0.2)
    fig.colorbar(scat, label="intensity (dB)")
    ttl = ax.set_title("")

    def update(i):
        p = clouds[i]
        if len(p):
            scat.set_offsets(np.c_[p[:, 0], p[:, 1]])
            scat.set_array(20 * np.log10(p[:, 2] + 1e-6))
        else:
            scat.set_offsets(np.empty((0, 2)))
            scat.set_array(np.array([]))
        ttl.set_text(f"{title}    frame {i + 1}/{len(clouds)}")
        return scat,

    anim = FuncAnimation(fig, update, frames=len(clouds), interval=1000.0 / fps, blit=False)

    writer = None
    if out.lower().endswith(".mp4"):
        try:
            import imageio_ffmpeg
            matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        except Exception:
            pass
        if FFMpegWriter.isAvailable():
            writer = FFMpegWriter(fps=fps)
        else:
            out = out[:-4] + ".gif"
            print("  ffmpeg not found -> GIF instead:", out, "(pip install imageio-ffmpeg for mp4)")
    if writer is None:
        writer = PillowWriter(fps=fps)
    print(f"  encoding {out} ...", flush=True)
    anim.save(out, writer=writer)
    plt.close(fig)
    print("  saved", out)


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    path = sys.argv[1]
    fps, db, ext = 20.0, 12.0, "mp4"
    for i, a in enumerate(sys.argv):
        if a == "--fps": fps = float(sys.argv[i + 1])
        if a == "--db":  db  = float(sys.argv[i + 1])
        if a == "--ext": ext = sys.argv[i + 1].lstrip(".")

    try:
        ref = np.load("reference_48ch.npy")
        cal = np.conj(ref) / (np.abs(ref) + 1e-9)
        print("loaded reference_48ch.npy (will make calibrated movie too)")
    except FileNotFoundError:
        cal = None
        print("reference_48ch.npy not found -> making the no-calibration movie only")

    nocal_frames, cal_frames = [], []
    for i, c in enumerate(frames(path)):
        nc, wc = clouds_both(c, cal, db)
        nocal_frames.append(nc); cal_frames.append(wc)
        print(f"\r  building frame {i + 1}", end="", flush=True)
    print()

    render(nocal_frames, f"topdown_nocal.{ext}", fps, "no calibration")
    if cal is not None:
        render(cal_frames, f"topdown_cal.{ext}", fps, "with calibration")


if __name__ == "__main__":
    main()
