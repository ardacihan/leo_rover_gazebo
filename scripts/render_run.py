"""Render a recorded exploration run into per-frame debug images + GIF.

Usage: python render_run.py <run_dir> [--every N] [--gif]

For each recorder frame draws three panels:
  1. SLAM map      - free/unknown/occupied, RED DOTS on occupied cells
  2. global costmap - orange penalty gradient, RED DOTS on lethal cells
  3. local costmap  - same treatment, robot-centred window
overlaid with the path so far, current pose, Nav2 plan, frontier centroids
and the active goal. Output lands in <run_dir>/render/.

The red dots are the actual obstacle cells the inflation layer grows its
penalty halos from - when a wall looks fat in panel 2, panel 1 shows whether
that is a real wall or a lone speckle cell carrying a halo.
"""
import argparse
import glob
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

COST_CMAP = LinearSegmentedColormap.from_list(
    "penalty", ["#ffffff", "#ffe0b0", "#ffb347", "#ff8c00", "#d95f02"])


def draw_map(ax, grid, meta, path, pose, plan, fr, goal, title, kind):
    ox, oy, res = meta
    h, w = grid.shape
    ext = [ox, ox + w * res, oy, oy + h * res]
    if kind == "map":
        img = np.full(grid.shape, 0.94)
        img[grid == -1] = 0.72
        img[grid >= 65] = 0.15
        ax.imshow(img, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1,
                  interpolation="nearest")
        oy_i, ox_i = np.where(grid >= 65)
    else:
        g = np.where(grid == -1, np.nan, grid.astype(float))
        ax.imshow(g, origin="lower", extent=ext, cmap=COST_CMAP, vmin=0, vmax=100,
                  interpolation="nearest")
        ax.imshow(np.where(grid == -1, 0.72, np.nan), origin="lower", extent=ext,
                  cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        oy_i, ox_i = np.where(grid >= 100)  # true lethal cells only; 99=inscribed shows as dark orange
    if len(ox_i):
        ax.plot(ox + (ox_i + 0.5) * res, oy + (oy_i + 0.5) * res, ".",
                color="#d40000", ms=2.5, zorder=3, label="obstacle cells")
    if path is not None and len(path):
        ax.plot(path[:, 0], path[:, 1], "-", color="#1f6fb5", lw=1.4, zorder=4)
    if plan is not None and len(plan):
        ax.plot(plan[:, 0], plan[:, 1], "--", color="#00a0a0", lw=1.1, zorder=4)
    if fr is not None and len(fr):
        ax.plot(fr[:, 0], fr[:, 1], "*", color="#7a00c9", ms=11, zorder=5)
    if goal is not None:
        ax.plot(goal[0], goal[1], "x", color="#009e2f", ms=11, mew=2.5, zorder=6)
    if pose is not None and np.isfinite(pose[0]):
        ax.arrow(pose[0], pose[1], 0.3 * np.cos(pose[2]), 0.3 * np.sin(pose[2]),
                 head_width=0.12, color="#1f6fb5", zorder=6)
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--every", type=int, default=5, help="render every Nth frame")
    ap.add_argument("--gif", action="store_true", help="assemble animated GIF")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.run_dir, "frame_*.npz")))
    if not files:
        raise SystemExit(f"no frames in {args.run_dir}")
    out = os.path.join(args.run_dir, "render")
    os.makedirs(out, exist_ok=True)

    frames = [np.load(f) for f in files]
    t0 = frames[0]["t"][0]
    poses = np.array([d["pose"][:2] for d in frames])
    # fixed world window over the whole run so panels don't jump around
    d_last = frames[-1]
    ox, oy, res = d_last["map_meta"]
    h, w = d_last["map"].shape
    xlim = (ox, ox + w * res)
    ylim = (oy, oy + h * res)

    rendered = []
    for i in range(0, len(frames), args.every):
        d = frames[i]
        t = d["t"][0] - t0
        pose = d["pose"]
        plan = d["plan"] if "plan" in d.files else None
        fr = d["frontier_centroids"] if "frontier_centroids" in d.files else None
        goal = d["goal"] if "goal" in d.files else None
        path = poses[: i + 1]
        fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))
        draw_map(axes[0], d["map"], d["map_meta"], path, pose, plan, fr, goal,
                 f"t={t:5.0f}s  SLAM map (red = occupied cells)", "map")
        axes[0].set_xlim(xlim); axes[0].set_ylim(ylim)
        if "gcost" in d.files:
            draw_map(axes[1], d["gcost"], d["gcost_meta"], path, pose, plan, fr,
                     goal, "global costmap (red = lethal, orange = penalty)", "cost")
            axes[1].set_xlim(xlim); axes[1].set_ylim(ylim)
        if "lcost" in d.files:
            draw_map(axes[2], d["lcost"], d["lcost_meta"], None, pose, plan,
                     None, goal, "local costmap (rolling)", "cost")
        fig.tight_layout()
        name = os.path.join(out, f"debug_{i:05d}.png")
        fig.savefig(name, dpi=95)
        plt.close(fig)
        rendered.append(name)
        print("wrote", name)

    if args.gif and rendered:
        from PIL import Image
        imgs = [Image.open(p).convert("P", palette=Image.ADAPTIVE) for p in rendered]
        gif = os.path.join(out, "run.gif")
        imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=450, loop=0)
        print("wrote", gif)


if __name__ == "__main__":
    main()
