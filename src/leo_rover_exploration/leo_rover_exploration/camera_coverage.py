"""Camera coverage tracking for item search.

The lidar builds the map, but markers/items are found by the RGBD camera,
which has a narrow FOV and a short practical detection range. This module
tracks which wall cells the camera has actually observed (frustum raycast on
the occupancy grid) so exploration can finish only when the *camera* has
swept the environment, not just the lidar.

Observed cells are keyed by world-quantized coordinates, so the tracker is
immune to SLAM growing or re-anchoring the map.
"""

import math
from collections import deque

import numpy as np
from nav_msgs.msg import OccupancyGrid

FREE_MAX = 50
UNKNOWN = -1


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CameraCoverageTracker:

    def __init__(self, fov=1.047, detection_range=3.0, n_rays=60):
        self.fov = fov
        self.detection_range = detection_range
        self.n_rays = n_rays
        self.observed = set()   # world-quantized (i, j) wall cells

    # ------------------------------------------------------------- update

    def update(self, map_msg, cam_x, cam_y, cam_yaw):
        """Cast the camera frustum onto the map, mark hit walls observed."""
        self.observed.update(
            self._raycast_hits(map_msg, cam_x, cam_y, cam_yaw))

    def estimate_gain(self, map_msg, cam_x, cam_y, cam_yaw):
        """Return how many currently unseen wall cells a pose would observe."""
        hits = self._raycast_hits(map_msg, cam_x, cam_y, cam_yaw)
        return len(hits.difference(self.observed))

    def _raycast_hits(self, map_msg, cam_x, cam_y, cam_yaw):
        """Return occupied wall keys hit by a camera frustum without mutation."""
        info = map_msg.info
        grid = np.asarray(map_msg.data, dtype=np.int8).reshape(
            info.height, info.width)
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        step = res * 0.7
        n_steps = int(self.detection_range / step)
        hits = set()

        for theta in np.linspace(cam_yaw - self.fov / 2.0,
                                 cam_yaw + self.fov / 2.0, self.n_rays):
            dx, dy = math.cos(theta) * step, math.sin(theta) * step
            x, y = cam_x, cam_y
            for _ in range(n_steps):
                x += dx
                y += dy
                c = int((x - ox) / res)
                r = int((y - oy) / res)
                if r < 0 or c < 0 or r >= info.height or c >= info.width:
                    break
                v = grid[r, c]
                if v >= FREE_MAX:
                    hits.add(self._key(x, y, res))
                    break
                # free or unknown: the camera can see across unmapped floor
        return hits

    @staticmethod
    def _key(x, y, res):
        return (int(math.floor(x / res)), int(math.floor(y / res)))

    def is_observed(self, map_msg, xy):
        return self._key(xy[0], xy[1], map_msg.info.resolution) in self.observed

    # -------------------------------------------------------------- stats

    def _walls_of_interest(self, map_msg):
        """Occupied cells adjacent to free space (where items could hang)."""
        info = map_msg.info
        grid = np.asarray(map_msg.data, dtype=np.int8).reshape(
            info.height, info.width)
        occ = grid >= FREE_MAX
        free = (grid >= 0) & (grid < FREE_MAX)
        near_free = np.zeros_like(free)
        near_free[1:, :] |= free[:-1, :]
        near_free[:-1, :] |= free[1:, :]
        near_free[:, 1:] |= free[:, :-1]
        near_free[:, :-1] |= free[:, 1:]
        return occ & near_free, grid

    def stats(self, map_msg):
        walls, _ = self._walls_of_interest(map_msg)
        info = map_msg.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        cells = np.argwhere(walls)
        if cells.size == 0:
            return 0, 0, 1.0
        observed = 0
        for r, c in cells:
            x = ox + (c + 0.5) * res
            y = oy + (r + 0.5) * res
            if self._key(x, y, res) in self.observed:
                observed += 1
        total = len(cells)
        return observed, total, observed / total

    def unobserved_clusters(self, map_msg, min_cells=5, max_cells=60):
        """Cluster unobserved wall cells.

        Returns [{'n_cells', 'centroid', 'target'}]. `target` is the member
        cell nearest the centroid: for elongated or L-shaped clusters the
        centroid itself can sit on open floor (or past detection range), so
        aiming the camera at it observes nothing — always aim at `target`.
        """
        walls, _ = self._walls_of_interest(map_msg)
        info = map_msg.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        unobserved = set()
        for r, c in np.argwhere(walls):
            x = ox + (c + 0.5) * res
            y = oy + (r + 0.5) * res
            if self._key(x, y, res) not in self.observed:
                unobserved.add((int(r), int(c)))

        clusters = []
        while unobserved:
            seed = unobserved.pop()
            queue = deque([seed])
            members = [seed]
            # Office and room perimeters are often one connected occupied
            # component. Treating the full perimeter as one cluster produces
            # one centroid target and lets a single strike suppress metres of
            # unseen wall. Segment it into camera-sized connected chunks.
            while queue and len(members) < max_cells:
                r, c = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in unobserved:
                            if len(members) >= max_cells:
                                break
                            unobserved.remove(n)
                            queue.append(n)
                            members.append(n)
                    if len(members) >= max_cells:
                        break
            if len(members) < min_cells:
                continue
            pts = np.array(members, dtype=float)
            cy, cx = pts.mean(axis=0)
            tr, tc = min(members, key=lambda m: (m[0] - cy) ** 2
                         + (m[1] - cx) ** 2)
            clusters.append({
                'n_cells': len(members),
                'centroid': (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res),
                'target': (ox + (tc + 0.5) * res, oy + (tr + 0.5) * res),
            })
        return clusters

    def find_viewpoint(self, map_msg, target, robot,
                       min_dist=0.6, clearance=3):
        """Free cell with line of sight to `target` within detection range.

        Samples rings around the target and returns the candidate closest to
        the robot, oriented to face the target. Returns (x, y) or None.
        """
        candidates = self.viewpoint_candidates(
            map_msg, target, min_dist=min_dist, clearance=clearance)
        if not candidates:
            return None
        return min(candidates, key=lambda p: math.hypot(
            p[0] - robot[0], p[1] - robot[1]))

    def viewpoint_candidates(self, map_msg, target,
                             min_dist=0.6, clearance=3):
        """Return free, line-of-sight camera poses around a wall target.

        Keeping all valid rings lets the caller trade travel against actual
        observable gain. Choosing only the closest pose often parked the
        camera 0.6 m from a wall, where its FOV covered very few wall cells.
        """
        info = map_msg.info
        grid = np.asarray(map_msg.data, dtype=np.int8).reshape(
            info.height, info.width)
        free = (grid >= 0) & (grid < FREE_MAX)
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        def cell_ok(x, y):
            c = int((x - ox) / res)
            r = int((y - oy) / res)
            k = clearance
            if r < k or c < k or r >= info.height - k or c >= info.width - k:
                return False
            return bool(free[r - k:r + k + 1, c - k:c + k + 1].all())

        def line_of_sight(x0, y0, x1, y1):
            d = math.hypot(x1 - x0, y1 - y0)
            steps = max(int(d / (res * 0.7)), 1)
            for i in range(1, steps):
                x = x0 + (x1 - x0) * i / steps
                y = y0 + (y1 - y0) * i / steps
                c = int((x - ox) / res)
                r = int((y - oy) / res)
                if r < 0 or c < 0 or r >= info.height or c >= info.width:
                    return False
                if grid[r, c] >= FREE_MAX:
                    return False
            return True

        candidates = []
        # 0.8: leave headroom for xy_goal_tolerance and the camera's offset
        # from base_link so a "reached" viewpoint still has the target in
        # actual frustum range.
        for radius in np.arange(min_dist, self.detection_range * 0.8, 0.4):
            for ang in np.arange(0.0, 2 * math.pi, math.pi / 6):
                x = target[0] + radius * math.cos(ang)
                y = target[1] + radius * math.sin(ang)
                if not cell_ok(x, y):
                    continue
                if not line_of_sight(x, y, target[0], target[1]):
                    continue
                candidates.append((x, y))
        return candidates

    # ------------------------------------------------------ visualization

    def grid_msg(self, map_msg, frame_id='map'):
        """Coverage as OccupancyGrid: unobserved walls 100, observed 0."""
        walls, _ = self._walls_of_interest(map_msg)
        info = map_msg.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.full(walls.shape, UNKNOWN, dtype=np.int8)
        for r, c in np.argwhere(walls):
            x = ox + (c + 0.5) * res
            y = oy + (r + 0.5) * res
            data[r, c] = 0 if self._key(x, y, res) in self.observed else 100
        out = OccupancyGrid()
        out.header.frame_id = frame_id
        out.header.stamp = map_msg.header.stamp
        out.info = info
        out.data = data.flatten().tolist()
        return out
