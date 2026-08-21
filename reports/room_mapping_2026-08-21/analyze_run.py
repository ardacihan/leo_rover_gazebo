"""Reconstruct the 2026-08-21 exploration run from run_recorder frames."""
import glob
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.join(HERE, "run_2026-08-21_1")
OUT = HERE

frames = []
for f in sorted(glob.glob(os.path.join(RUN, "frame_*.npz"))):
    d = np.load(f)
    frames.append(d)

t0 = frames[0]["t"][0]
ts = np.array([d["t"][0] - t0 for d in frames])
poses = np.array([d["pose"] for d in frames])
known = np.array([(d["map"] != -1).sum() if "map" in d else 0 for d in frames])
free = np.array([(d["map"] == 0).sum() if "map" in d else 0 for d in frames])
nfront = np.array(
    [len(d["frontier_centroids"]) if "frontier_centroids" in d.files else -1 for d in frames]
)

events = []
with open(os.path.join(RUN, "events.jsonl")) as fh:
    for line in fh:
        events.append(json.loads(line))
gs = [e for e in events if e.get("event") == "goal_status"]
print(f"frames={len(frames)} span={ts[-1]:.0f}s goal_events={len(gs)}")
st_counts = {}
for e in gs:
    st_counts[e["status"]] = st_counts.get(e["status"], 0) + 1
print("status counts:", st_counts)
# goal event rate over time (bucketed 10 s)
et = np.array([e["t"] - t0 for e in gs])

# ---- timeline figure ----
fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
axes[0].plot(ts, known * 0.0025, label="known m2")
axes[0].plot(ts, free * 0.0025, label="free m2")
axes[0].set_ylabel("area m$^2$")
axes[0].legend()
axes[0].set_title("map growth")
d = np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1]))
d = np.insert(np.nancumsum(d), 0, 0)
axes[1].plot(ts, d)
axes[1].set_ylabel("dist m")
axes[1].set_title("cumulative distance travelled")
axes[2].plot(ts, nfront, ".-")
axes[2].set_ylabel("# frontiers")
axes[2].set_title("frontier centroids visible")
axes[3].hist(et, bins=np.arange(0, ts[-1] + 10, 10))
axes[3].set_ylabel("goal events /10s")
axes[3].set_title("NavigateToPose status events")
axes[3].set_xlabel("t since recorder start [s]")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "timeline.png"), dpi=110)
print("wrote timeline.png")


def draw_grid(ax, grid, meta, title, pose=None, path=None, plan=None, fr=None, goal=None):
    ox, oy, res = meta
    h, w = grid.shape
    extent = [ox, ox + w * res, oy, oy + h * res]
    g = grid.astype(float)
    ax.imshow(
        np.where(g == -1, np.nan, g), origin="lower", extent=extent, cmap="viridis",
        vmin=0, vmax=100, interpolation="nearest",
    )
    ax.imshow(
        np.where(g == -1, 0.5, np.nan), origin="lower", extent=extent, cmap="gray",
        vmin=0, vmax=1, interpolation="nearest",
    )
    if path is not None:
        ax.plot(path[:, 0], path[:, 1], "w-", lw=1.2, label="path so far")
    if plan is not None and len(plan):
        ax.plot(plan[:, 0], plan[:, 1], "c--", lw=1, label="nav2 plan")
    if fr is not None and len(fr):
        ax.plot(fr[:, 0], fr[:, 1], "r*", ms=12, label="frontier")
    if goal is not None:
        ax.plot(goal[0], goal[1], "yx", ms=10, mew=2, label="goal")
    if pose is not None and not np.isnan(pose[0]):
        ax.arrow(pose[0], pose[1], 0.25 * np.cos(pose[2]), 0.25 * np.sin(pose[2]),
                 head_width=0.09, color="orange")
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal")


# ---- map snapshots over time ----
picks = [int(len(frames) * f) for f in (0.1, 0.3, 0.5, 0.7, 0.85, 0.99)]
fig, axes = plt.subplots(2, 3, figsize=(15, 9))
for ax, i in zip(axes.flat, picks):
    d = frames[i]
    fr = d["frontier_centroids"] if "frontier_centroids" in d.files else None
    goal = d["goal"] if "goal" in d.files else None
    plan = d["plan"] if "plan" in d.files else None
    draw_grid(ax, d["map"], d["map_meta"], f"t={ts[i]:.0f}s  SLAM map", pose=d["pose"],
              path=poses[: i + 1], plan=plan, fr=fr, goal=goal)
axes.flat[0].legend(fontsize=7, loc="lower right")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "map_evolution.png"), dpi=110)
print("wrote map_evolution.png")

# ---- final costmaps ----
last = None
for d in reversed(frames):
    if "gcost" in d.files:
        last = d
        break
fig, axes = plt.subplots(1, 3, figsize=(16, 6))
dl = frames[-1]
fr = dl["frontier_centroids"] if "frontier_centroids" in dl.files else None
goal = dl["goal"] if "goal" in dl.files else None
draw_grid(axes[0], dl["map"], dl["map_meta"], f"final SLAM map t={ts[-1]:.0f}s",
          pose=dl["pose"], path=poses, fr=fr, goal=goal)
if last is not None:
    draw_grid(axes[1], last["gcost"], last["gcost_meta"], "final global costmap",
              pose=last["pose"], path=poses, fr=fr, goal=goal)
    if "lcost" in last.files:
        draw_grid(axes[2], last["lcost"], last["lcost_meta"], "final local costmap",
                  pose=last["pose"])
fig.tight_layout()
fig.savefig(os.path.join(OUT, "final_costmaps.png"), dpi=110)
print("wrote final_costmaps.png")

# where is unknown adjacent to free (frontier cells by hand) in final map?
m = dl["map"][0] if isinstance(dl["map"], tuple) else dl["map"]
meta = dl["map_meta"]
freem = m == 0
unk = m == -1
adj = np.zeros_like(freem)
adj[1:, :] |= freem[1:, :] & unk[:-1, :]
adj[:-1, :] |= freem[:-1, :] & unk[1:, :]
adj[:, 1:] |= freem[:, 1:] & unk[:, :-1]
adj[:, :-1] |= freem[:, :-1] & unk[:, 1:]
ys, xs = np.where(adj)
print(f"final map: {adj.sum()} frontier cells (free touching unknown)")
if len(xs):
    wx = meta[0] + xs * meta[2]
    wy = meta[1] + ys * meta[2]
    print("frontier cell extent x:[%.2f %.2f] y:[%.2f %.2f]" % (wx.min(), wx.max(), wy.min(), wy.max()))
