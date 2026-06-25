"""
background_stability.py  --  is the static scene actually static?

Looks at ONE virtual channel (one TX-RX pair) over a whole capture and builds a
range-vs-time image. For each frame: range-FFT the chosen channel, average the
64 loops to denoise, keep the resulting range profile. Stack every frame's
profile side by side over time.

    python background_stability.py capture_xxxx.bin [start_frame] [tx] [rx]
        start_frame  first frame to use   (default 0; skip a noisy warm-up)
        tx           TX slot 0..5          (default 0)
        rx           RX index 0..7         (default 0)

If the room is truly still you should see HORIZONTAL bars: each range bin holds
constant magnitude across time. Flicker / drift / fading at a range bin means
that range was not stable, and averaging it into a background will be dirty.

Reads the file with mmap, so a multi-minute (multi-GB) capture is fine.
Saves background_stability.png.
"""
import sys
import mmap
import numpy as np
import angle_estimation_offline as ae

DYN_RANGE_DB = 40.0        # color span below the peak


def chirp_data_offsets(mm):
    """Data-start byte offset of every chirp, in order (header sits after data)."""
    starts = []
    i = mm.find(ae.MAGIC_LE)
    while i != -1:
        if i >= ae.DATA_PER_CHIRP:
            starts.append(i - ae.DATA_PER_CHIRP)
        i = mm.find(ae.MAGIC_LE, i + ae.HEADER_BYTES)
    return starts


def channel_profile(mm, offsets_for_tx, rx, win):
    """64 loop chirps for one TX -> one [SAMPLES] range profile for (tx, rx)."""
    loops = np.empty((len(offsets_for_tx), ae.SAMPLES), complex)
    for k, off in enumerate(offsets_for_tx):
        buf = np.frombuffer(mm, np.uint8, ae.DATA_PER_CHIRP, off)
        iq  = buf.view(np.int16).astype(np.float32)
        c   = iq[0::2] + 1j * iq[1::2]                 # 2048 complex
        c   = c.reshape(ae.NUM_RX, ae.SAMPLES)         # [rx, sample]
        loops[k] = c[rx]
    rng = np.fft.fft(loops * win[None, :], axis=1)     # range FFT per loop
    return np.abs(rng).mean(axis=0)                    # denoise across the 64 loops


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

    starts = chirp_data_offsets(mm)
    n_frames = len(starts) // ae.CHIRPS_PER_FRAME
    if start >= n_frames:
        raise SystemExit(f"file has only {n_frames} full frames; start={start} too large")

    win = np.hanning(ae.SAMPLES).astype(np.float32)
    n_use = n_frames - start
    M = np.empty((n_use, ae.SAMPLES))                  # [frame, range]

    for j in range(n_use):
        f0 = (start + j) * ae.CHIRPS_PER_FRAME
        frame_offsets = starts[f0:f0 + ae.CHIRPS_PER_FRAME]
        tx_offsets = frame_offsets[tx::ae.NUM_TX]      # this TX's 64 loop chirps
        M[j] = channel_profile(mm, tx_offsets, rx, win)

    mm.close(); f.close()

    print(f"frames in file:        {n_frames}")
    print(f"frames analyzed:       {n_use}  (from frame {start})")
    print(f"channel:               TX{tx} RX{rx}")
    print(f"per-frame channel cube: [{ae.NUM_LOOPS} loops x {ae.SAMPLES} samples]")
    print(f"stacked image:          [{n_use} frames x {ae.SAMPLES} range bins]")

    # ---- range-vs-time image: range on Y, time on X -> horizontal bars if stable
    img = 20.0 * np.log10(M.T + 1e-6)                  # [range, frame]
    peak = img.max()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    r_max = ae.range_bin_to_m(ae.SAMPLES - 1)
    fig, ax = plt.subplots(figsize=(11, 5))
    im = ax.imshow(img, aspect="auto", origin="lower", cmap="viridis",
                   extent=[start, n_frames, 0.0, r_max],
                   vmin=peak - DYN_RANGE_DB, vmax=peak)
    ax.set_xlabel("frame index  (time ->)")
    ax.set_ylabel("range (m)")
    ax.set_title(f"background stability  --  TX{tx} RX{rx}  "
                 f"(horizontal bars = stable)")
    fig.colorbar(im, ax=ax, label="magnitude (dB)")
    fig.tight_layout()
    fig.savefig("background_stability.png", dpi=120)
    print("saved background_stability.png")


if __name__ == "__main__":
    main()
