"""Distributed coordinated frontier allocation for multi-robot exploration.

Pure functions (no ROS deps) so the policy can be unit-tested in isolation.

The coordination is *distributed with no central node*: every rover shares the
same merged map (multirobot_map_merge) and the same TF tree, so each rover can
see all rovers' poses and detect the same frontiers. Each rover independently
runs the identical deterministic greedy assignment below and then follows its
own assigned frontier. Two mechanisms make two rovers explore *different*
regions instead of both chasing the single globally-best frontier:

  1. Global greedy assignment over all (rover, frontier) pairs by utility
     (information gain minus travel cost) - the classic Burgard et al. (2005)
     coordinated exploration objective, so the better-placed rover wins each
     frontier.
  2. A proximity discount: once a frontier is assigned, nearby frontiers lose
     utility for the remaining rovers, pushing them toward disjoint areas.

Peers' *committed* goals (from the shared claim topic) seed the assignment so
the policy respects what a peer is actually already driving toward, rather than
relying on both rovers recomputing byte-identical frontier sets.
"""

import math


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def base_utility(gain, robot_xy, frontier_xy, potential_scale, gain_scale,
                 mode='linear', min_travel_cost=1.0,
                 information_gain_travel_weight=0.10, dist=None):
    """Single-robot frontier score.

    'linear' is the original reward-minus-distance. 'ratio' is gain per metre
    driven, which is what keeps a large distant region competitive: under the
    linear form a frontier's reward has to exceed potential_scale times the
    extra distance, so a far room is never chosen while any nearer scrap
    survives, however little that scrap is worth.

    `dist`, when given, is a precomputed travel distance (geodesic through
    free space); the euclidean fallback lies through walls in maze maps.
    """
    d = _dist(robot_xy, frontier_xy) if dist is None else dist
    if mode == 'ratio':
        return gain / max(d, min_travel_cost)
    if mode == 'information_gain':
        # Unknown area is the objective; distance is only a modest cost. This
        # keeps a doorway into a room competitive with nearby map scraps while
        # avoiding gratuitous cross-building travel between equal-gain goals.
        return gain - information_gain_travel_weight * d
    return gain_scale * gain - potential_scale * d


def _normalize(frontiers):
    out = []
    for f in frontiers:
        if isinstance(f, dict):
            # An explicit information gain wins over the frontier's own
            # width when the caller computed one; size_m stays the fallback
            # so existing callers and tests are unaffected.
            gain = float(f.get('gain', f.get('size_m', 1.0)))
            out.append({'goal': tuple(f['goal']), 'gain': gain,
                        'tie_breaker': float(f.get('tie_breaker', 0.0))})
        else:
            out.append({'goal': (f[0], f[1]), 'gain': float(f[2]),
                        'tie_breaker': 0.0})
    return out


def _apply_discount(gains, used, frontiers, goal, radius, strength):
    """Damp the gain of not-yet-assigned frontiers near an assigned goal."""
    if radius <= 0.0 or strength <= 0.0:
        return
    sigma2 = 2.0 * (radius / 2.0) ** 2
    for i, f in enumerate(frontiers):
        if used[i]:
            continue
        d = _dist(goal, f['goal'])
        if d < radius:
            gains[i] = max(0.0, gains[i] * (1.0 - strength * math.exp(-(d * d) / sigma2)))


def coordinated_allocation(robots, frontiers, committed=None,
                           potential_scale=3.0, gain_scale=1.0,
                           discount_radius=3.0, discount_strength=1.0,
                           utility_mode='linear', min_travel_cost=1.0,
                           information_gain_travel_weight=0.10,
                           dist_lookup=None):
    """Assign at most one frontier to each robot.

    robots:    list of (name, (x, y)); MUST include self.
    frontiers: list of {'goal': (x, y), 'size_m': float} or (x, y, gain).
    committed: optional dict name -> (x, y) of peers' already-committed goals;
               each is pre-assigned to the nearest frontier so the greedy
               respects reality. Robots in `committed` should also be in
               `robots`.
    Returns dict name -> assigned goal (x, y), or None if the robot got none
    (more robots than frontiers). Deterministic: ties break by (name, goal).
    """
    F = _normalize(frontiers)
    assignment = {name: None for name, _ in robots}
    if not F:
        return assignment

    gains = [f['gain'] for f in F]
    used = [False] * len(F)
    remaining = list(robots)

    # Seed with peers' committed goals: lock the nearest frontier to each
    # commitment so we plan around what peers are actually doing.
    committed = committed or {}
    for name, cxy in committed.items():
        if name not in assignment:
            continue
        avail = [i for i in range(len(F)) if not used[i]]
        if not avail:
            continue
        fi = min(avail, key=lambda i: (_dist(cxy, F[i]['goal']), F[i]['goal']))
        assignment[name] = F[fi]['goal']
        used[fi] = True
        remaining = [(n, xy) for n, xy in remaining if n != name]
        _apply_discount(gains, used, F, F[fi]['goal'],
                        discount_radius, discount_strength)

    # Greedy global assignment for the rest.
    while remaining and any(not u for u in used):
        best = None  # (geometric utility, soft tie-breaker, name, goal, fidx)
        for name, xy in remaining:
            for fi, f in enumerate(F):
                if used[fi]:
                    continue
                d = dist_lookup(name, f['goal']) if dist_lookup else None
                u = base_utility(gains[fi], xy, f['goal'],
                                 potential_scale, gain_scale,
                                 utility_mode, min_travel_cost,
                                 information_gain_travel_weight, dist=d)
                hint = f['tie_breaker']
                if best is None or u > best[0] or (
                        u == best[0] and hint > best[1]) or (
                        u == best[0] and hint == best[1]
                        and (name, f['goal']) < (best[2], best[3])):
                    best = (u, hint, name, f['goal'], fi)
        if best is None:
            break
        _, _, name, goal, fi = best
        assignment[name] = goal
        used[fi] = True
        remaining = [(n, xy) for n, xy in remaining if n != name]
        _apply_discount(gains, used, F, goal,
                        discount_radius, discount_strength)

    return assignment


def independent_choice(robot_xy, frontiers, potential_scale=3.0, gain_scale=1.0):
    """Uncoordinated baseline: pick the single best frontier for this robot,
    ignoring peers. Two robots running this converge on the same frontier."""
    F = _normalize(frontiers)
    if not F:
        return None
    best = max(
        F, key=lambda f: (base_utility(f['gain'], robot_xy, f['goal'],
                                       potential_scale, gain_scale),
                          tuple(-c for c in f['goal'])))
    return best['goal']
