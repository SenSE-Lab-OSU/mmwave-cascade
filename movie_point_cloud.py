"""
movie_point_cloud.py -- animate the 3D point cloud across every frame of a capture.

    python movie_point_cloud.py capture_xxxx.bin
    python movie_point_cloud.py capture_xxxx.bin --out cloud.gif --fps 10 --db 12 --rmax 8

Writes a movie (GIF by default). Pass --out name.mp4 if you have ffmpeg installed.
Calibration is automatic when reference_48ch.npy sits in the folder (it rides along
through angle_estimation_offline).
"""
import sys
import numpy as np
import angle_estimation_offline as ae


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


def cloud(cube, db=12.0):
    """Point cloud for one frame -> array of (x, y, z, amp)."""
    rng      = ae.range_fft(cube)
    dop, mag = ae.range_doppler(rng)
    cells    = ae.detect(mag, db_below_peak=db)
    CAL = getattr(ae, "CAL", None)
    pts = []
    for dbin, rbin in cells:
        snap = dop[dbin, :, :, rbin]
        if CAL is not None:
            snap = snap * CAL
        az, el = ae.estimate_angles(ae.snapshot_to_array(snap))
        R = ae.range_bin_to_m(rbin)
        x, y, z = ae.to_xyz(R, az, el)
        pts.append((x, y, z, float(mag[dbin, rbin])))
    return np.array(pts) if pts else np.zeros((0, 4))


def make_movie(path, out="point_cloud.gif", fps=10, db=12.0, rmax=8.0):
    clouds = []
    for i, c in enumerate(frames(path)):
        clouds.append(cloud(c, db))
        print(f"\r  building clouds: frame {i + 1}", end="", flush=True)
    print()
    if not clouds:
        raise SystemExit("no frames found in file")

    allpts = np.vstack([p for p in clouds if len(p)]) if any(len(p) for p in clouds) \
        else np.zeros((1, 4))
    xlo, xhi = allpts[:, 0].min() - 0.5, allpts[:, 0].max() + 0.5
    zlo, zhi = allpts[:, 2].min() - 0.5, allpts[:, 2].max() + 0.5
    if xhi - xlo < 1: xlo, xhi = xlo - 1, xhi + 1
    if zhi - zlo < 1: zlo, zhi = zlo - 1, zhi + 1
    dbvals = 20 * np.log10(allpts[:, 3] + 1e-6)
    vmax = float(dbvals.max()); vmin = vmax - 30

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    scat = ax.scatter([0], [0], [0], c=[vmin], cmap="viridis",
                      vmin=vmin, vmax=vmax, s=18, depthshade=False)
    ax.set_xlabel("x  left-right (m)")
    ax.set_ylabel("y  forward (m)")
    ax.set_zlabel("z  up-down (m)")
    ax.set_xlim(xlo, xhi); ax.set_ylim(0, rmax); ax.set_zlim(zlo, zhi)
    ax.view_init(elev=22, azim=-72)
    fig.colorbar(scat, label="intensity (dB)", shrink=0.6)
    title = ax.set_title("")

    def update(i):
        p = clouds[i]
        if len(p):
            scat._offsets3d = (p[:, 0], p[:, 1], p[:, 2])
            scat.set_array(20 * np.log10(p[:, 3] + 1e-6))
        else:
            scat._offsets3d = ([], [], [])
            scat.set_array(np.array([]))
        title.set_text(f"frame {i + 1}/{len(clouds)}    {len(p)} pts")
        return scat,

    anim = FuncAnimation(fig, update, frames=len(clouds),
                         interval=1000.0 / fps, blit=False)
    print(f"  encoding {len(clouds)} frames to {out} (this stage is silent, give it a moment) ...",
          flush=True)
    from matplotlib.animation import FFMpegWriter, PillowWriter
    writer = None
    if out.lower().endswith(".mp4"):
        try:                                          # use a bundled ffmpeg if present
            import imageio_ffmpeg
            matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        except Exception:
            pass
        if FFMpegWriter.isAvailable():
            writer = FFMpegWriter(fps=fps)
        else:
            out = out[:-4] + ".gif"
            print("  ffmpeg not found -> writing GIF instead:", out)
            print("  (want mp4? run:  pip install imageio-ffmpeg )")
    if writer is None:
        writer = PillowWriter(fps=fps)
    anim.save(out, writer=writer)
    print(f"saved {out}   ({len(clouds)} frames)")


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    path = sys.argv[1]
    kw = {}
    for i, a in enumerate(sys.argv):
        if a == "--out":  kw["out"]  = sys.argv[i + 1]
        if a == "--fps":  kw["fps"]  = float(sys.argv[i + 1])
        if a == "--db":   kw["db"]   = float(sys.argv[i + 1])
        if a == "--rmax": kw["rmax"] = float(sys.argv[i + 1])
    make_movie(path, **kw)


if __name__ == "__main__":
    main()