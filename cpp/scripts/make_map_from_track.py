#!/usr/bin/env python3

import argparse
import os
from collections import deque


def save_pgm(path: str, img_u8) -> None:
    # P5 binary PGM
    h, w = img_u8.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(img_u8.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate an occupancy map (PGM) from track.jpg")
    ap.add_argument("--in", dest="inp", default="track.jpg", help="Input track image (jpg/png).")
    ap.add_argument("--out", default="cpp/data/track_map.pgm", help="Output occupancy map (PGM).")
    ap.add_argument(
        "--free",
        choices=["dark", "bright"],
        default="dark",
        help="Whether free space corresponds to dark pixels (track asphalt) or bright pixels.",
    )
    ap.add_argument("--threshold", type=int, default=140, help="Grayscale threshold [0..255].")
    ap.add_argument("--blur", type=float, default=1.2, help="Gaussian blur sigma (0 disables).")
    ap.add_argument(
        "--seed",
        default="691,1125",
        help="Seed pixel (x,y) that is guaranteed to be on the track; used to keep only that connected component.",
    )
    ap.add_argument("--debug-out", default="cpp/data/track_map_debug.png", help="Debug overlay PNG output.")
    args = ap.parse_args()

    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except Exception as e:
        print("ERROR: numpy+matplotlib are required.")
        print(f"Reason: {e}")
        return 2

    img = plt.imread(args.inp)
    # plt.imread may return float in [0,1] or uint8 in [0,255]
    if img.dtype.kind == "f":
        img = (img * 255.0).clip(0, 255).astype("uint8")
    if img.ndim == 3:
        # RGB -> gray
        img = (0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]).astype("uint8")

    gray = img.astype("float32")
    if args.blur and args.blur > 0:
        # Simple separable gaussian blur without scipy
        sigma = float(args.blur)
        radius = int(max(1, round(3 * sigma)))
        x = np.arange(-radius, radius + 1, dtype=np.float32)
        k = np.exp(-(x * x) / (2.0 * sigma * sigma))
        k = k / np.sum(k)
        # horizontal
        tmp = np.pad(gray, ((0, 0), (radius, radius)), mode="edge")
        tmp = np.apply_along_axis(lambda r: np.convolve(r, k, mode="valid"), 1, tmp)
        # vertical
        tmp2 = np.pad(tmp, ((radius, radius), (0, 0)), mode="edge")
        gray = np.apply_along_axis(lambda c: np.convolve(c, k, mode="valid"), 0, tmp2)

    thr = int(args.threshold)
    if args.free == "dark":
        free = gray < thr
    else:
        free = gray > thr

    # Keep only the connected component that contains the seed pixel.
    try:
        sx_s, sy_s = args.seed.split(",")
        sx = int(float(sx_s))
        sy = int(float(sy_s))
    except Exception:
        print(f"ERROR: invalid --seed '{args.seed}' (expected 'x,y')")
        return 2

    h, w = free.shape
    sx = max(0, min(w - 1, sx))
    sy = max(0, min(h - 1, sy))
    if not free[sy, sx]:
        # Try a small local search for a free pixel near the seed.
        found = False
        for r in range(1, 25):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    x = sx + dx
                    y = sy + dy
                    if 0 <= x < w and 0 <= y < h and free[y, x]:
                        sx, sy = x, y
                        found = True
                        break
                if found:
                    break
            if found:
                break
        if not found:
            print("ERROR: seed is not on a free pixel and no nearby free pixel was found. Adjust --seed/--threshold.")
            return 2

    keep = np.zeros_like(free, dtype=bool)
    q = deque()
    q.append((sx, sy))
    keep[sy, sx] = True
    while q:
        x, y = q.popleft()
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= nx < w and 0 <= ny < h and (not keep[ny, nx]) and free[ny, nx]:
                keep[ny, nx] = True
                q.append((nx, ny))
    free = keep

    # Occupancy map convention in our C++: free if pixel > free_threshold (default 0).
    occ = (free.astype("uint8") * 255)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save_pgm(args.out, occ)

    # Debug output
    os.makedirs(os.path.dirname(args.debug_out), exist_ok=True)
    plt.figure(figsize=(10, 6))
    plt.subplot(1, 2, 1)
    plt.title("input (gray)")
    plt.imshow(img, cmap="gray")
    plt.axis("off")
    plt.subplot(1, 2, 2)
    plt.title("generated occupancy (white=free)")
    plt.imshow(occ, cmap="gray")
    plt.plot([sx], [sy], "rx", markersize=8)
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(args.debug_out, dpi=150)
    print(f"Wrote map: {args.out}")
    print(f"Wrote debug: {args.debug_out}")

    print("\nTo use this map in C++ (track image pixel space):")
    print(f"  map_file={args.out}")
    print("  map_free_threshold=0")
    print("  map_invert=false")
    print("  map_use_affine=true")
    print("  map_a11=0")
    print("  map_a12=360")
    print("  map_a21=390")
    print("  map_a22=0")
    print("  map_b1=687")
    print("  map_b2=1125")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

