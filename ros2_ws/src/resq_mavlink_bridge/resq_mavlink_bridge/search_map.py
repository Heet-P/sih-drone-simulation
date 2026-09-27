"""Search planning, hazard mapping and ground routing for the SAR mission.

No ROS or MAVLink here, so this can be tested and benchmarked on its own
(see resq_mavlink/search_benchmark.py). Everything is in a flat local
frame: metres east (e) and north (n) of the drone's launch point.

ProbabilityMap - Bayesian search theory, as used to plan real SAR
    searches. The area is split into cells, each holding the probability
    that the casualty is there. The prior comes from intel (debris fields
    and collapsed buildings, where people are more likely to be trapped,
    weigh more than open ground). Each time a cell passes through the
    camera footprint without a detection, its probability is multiplied
    by (1 - POD), the chance the detector would have missed a person
    there. The planner flies to wherever the most probability can be
    removed per second of flight, so the drone goes where the survivors
    most likely are instead of sweeping a fixed grid.

HazardMap - ground-projected hazard sightings (fire, flood water, known
    collapsed structures) merged into stable, geo-tagged hazard zones.

safe_route - A* over the hazard zones plus safety buffers, from the
    ground team's staging point to a casualty.
"""
import heapq
import math

import numpy as np


def camera_half_extents(alt, hfov, width, height):
    """Half the ground footprint of a nadir camera: (forward, right) metres."""
    focal = (width / 2.0) / math.tan(hfov / 2.0)
    return alt * (height / 2.0) / focal, alt * (width / 2.0) / focal


def footprint_corners(e, n, alt, yaw, hfov, width, height):
    """Ground corners of the camera footprint. yaw is the heading
    (radians, 0 = north, clockwise); image-up is the drone's nose."""
    half_fwd, half_right = camera_half_extents(alt, hfov, width, height)
    fe, fn = math.sin(yaw), math.cos(yaw)
    re, rn = math.cos(yaw), -math.sin(yaw)
    return [
        (e + sf * half_fwd * fe + sr * half_right * re, n + sf * half_fwd * fn + sr * half_right * rn)
        for sf, sr in ((1, -1), (1, 1), (-1, 1), (-1, -1))
    ]


class ProbabilityMap:

    def __init__(self, center, width, height, cell=2.0, zones=(), base=1.0, gain_radius=3.0, prior_map=None,
                 region_radius=0.0, region_weight=0.0):
        # gain_radius must fit inside the trusted part of the footprint
        # at any heading (3.3 m at 8 m altitude), or the planner can
        # hover over a spot whose "gain" it can never actually collect.
        self.nx = max(1, int(round(width / cell)))
        self.ny = max(1, int(round(height / cell)))
        self.width, self.height = width, height
        self.e0 = center[0] - width / 2.0
        self.n0 = center[1] - height / 2.0
        cw, ch = width / self.nx, height / self.ny
        es = self.e0 + (np.arange(self.nx) + 0.5) * cw
        ns = self.n0 + (np.arange(self.ny) + 0.5) * ch
        self.ee, self.nn = np.meshgrid(es, ns)  # shape (ny, nx), row = north

        if prior_map is not None:
            # A raster prior (e.g. built from pre-disaster land-use layers):
            # (grid, e0, n0, cell_m), sampled at this map's cell centres.
            grid, ge0, gn0, gcell = prior_map
            ci = np.clip(((self.ee - ge0) / gcell).astype(int), 0, grid.shape[1] - 1)
            ri = np.clip(((self.nn - gn0) / gcell).astype(int), 0, grid.shape[0] - 1)
            prior = grid[ri, ci].astype(float) + 1e-6
        else:
            prior = np.full(self.ee.shape, float(base))
        for zone in zones:
            d2 = (self.ee - zone['east_m']) ** 2 + (self.nn - zone['north_m']) ** 2
            prior += zone.get('weight', 1.0) * np.exp(-d2 / (2.0 * zone.get('radius_m', 5.0) ** 2))
        self.prior = prior / prior.sum()
        self.p = self.prior.copy()
        self.looks = np.zeros(self.ee.shape, dtype=int)

        self.centers = np.column_stack([self.ee.ravel(), self.nn.ravel()])
        # Gain of flying over a cell = probability within gain_radius of it,
        # computed as a small convolution over these cell offsets (an N x N
        # neighbour matrix would be ~680 MB for a 200 m area at 2 m cells).
        ri, rj = int(gain_radius // ch) + 1, int(gain_radius // cw) + 1
        self.kernel = [(di, dj) for di in range(-ri, ri + 1) for dj in range(-rj, rj + 1)
                       if (di * ch) ** 2 + (dj * cw) ** 2 <= gain_radius ** 2]
        self.kpad = (ri, rj)
        # Optional regional term for large areas: probability within
        # region_radius of a target (a box of that half-size), weighted by
        # region_weight, pulls the drone toward rich regions instead of
        # hopping between nearby leftover cells. The small term alone still
        # decides locally, so the drone never waits for gain it can't collect.
        self.region_cells = int(round(region_radius / cw)) if region_radius > 0 else 0
        self.region_weight = region_weight

    @property
    def pos(self):
        """Cumulative probability of success: how much of the prior
        probability of the casualty being here has been searched away."""
        return float(1.0 - self.p.sum())

    def observe(self, e, n, alt, yaw, pod, hfov, width, height, shrink=0.8):
        """A detector frame over this footprint found nobody. Only the
        central `shrink` of the frame counts - a person cut off at the
        edge is easy to miss. Returns the number of cells updated."""
        half_fwd, half_right = camera_half_extents(alt, hfov, width, height)
        de, dn = self.ee - e, self.nn - n
        fwd = de * math.sin(yaw) + dn * math.cos(yaw)
        right = de * math.cos(yaw) - dn * math.sin(yaw)
        mask = (np.abs(fwd) <= half_fwd * shrink) & (np.abs(right) <= half_right * shrink)
        self.p[mask] *= (1.0 - pod)
        self.looks[mask] += 1
        return int(mask.sum())

    def remove_disk(self, e, n, radius, keep=0.0):
        """A casualty was confirmed here: that probability is accounted for."""
        mask = (self.ee - e) ** 2 + (self.nn - n) ** 2 <= radius ** 2
        self.p[mask] *= keep

    def gains(self):
        ri, rj = self.kpad
        padded = np.pad(self.p, ((ri, ri), (rj, rj)))
        total = np.zeros_like(self.p)
        for di, dj in self.kernel:
            total += padded[ri + di:ri + di + self.ny, rj + dj:rj + dj + self.nx]
        if self.region_cells:
            k = self.region_cells
            # Box sums via an integral image.
            ii = np.pad(np.cumsum(np.cumsum(np.pad(self.p, k), 0), 1), ((1, 0), (1, 0)))
            box = ii[2 * k + 1:, 2 * k + 1:] - ii[:-2 * k - 1, 2 * k + 1:] - ii[2 * k + 1:, :-2 * k - 1] + ii[:-2 * k - 1, :-2 * k - 1]
            area_ratio = len(self.kernel) / float((2 * k + 1) ** 2)
            total = total + self.region_weight * area_ratio * box
        return total.ravel()

    def best_target(self, e, n, speed, overhead_s=4.0):
        """Cell to fly to next: most probability within reach of it per
        second of flight (travel time plus a fixed overhead, so nearby
        cells don't win on distance alone). Returns ((e, n), score)."""
        dist = np.hypot(self.centers[:, 0] - e, self.centers[:, 1] - n)
        score = self.gains() / (dist / speed + overhead_s)
        i = int(np.argmax(score))
        return (float(self.centers[i, 0]), float(self.centers[i, 1])), float(score[i])

    def score_at(self, target, e, n, speed, overhead_s=4.0):
        i = int(np.argmin(np.hypot(self.centers[:, 0] - target[0], self.centers[:, 1] - target[1])))
        dist = math.hypot(self.centers[i, 0] - e, self.centers[i, 1] - n)
        return float(self.gains()[i] / (dist / speed + overhead_s))

    def to_dict(self):
        # Scaled to the prior's peak, so drained cells read as dark and
        # untouched hot spots stay bright as the search goes on.
        scale = float(self.prior.max()) or 1.0
        return {
            'e0': round(self.e0, 2), 'n0': round(self.n0, 2),
            'w': self.width, 'h': self.height, 'nx': self.nx, 'ny': self.ny,
            'p': [round(float(v), 3) for v in (self.p / scale).ravel()],
        }


HAZARD_TYPES = {
    # label, safety buffer (m) the ground route keeps from the zone's edge
    'fire': ('Fire / hot spot', 4.0),
    'flood': ('Flood water', 1.5),
    'structure': ('Collapsed structure', 3.0),
}


# Sightings of the same kind closer than this are one hazard. A fire is seen
# from many angles and altitudes (the drone climbs over the burning block's
# neighbours), and at 3 m one burning block in the region world became 15
# zones spread over ~15 m.
MERGE_M = {'fire': 8.0, 'flood': 5.0, 'structure': 3.0}


class HazardMap:
    """Merges repeated sightings of a hazard into one zone. A zone is
    only reported once seen `confirm_hits` times, so a single bad frame
    or projection doesn't put a hazard on the map."""

    def __init__(self, confirm_hits=2, max_radius=10.0):
        self.zones = []
        self.confirm_hits = confirm_hits
        self.max_radius = max_radius

    def add(self, kind, e, n, radius, t, peak_k=None, conf=None, sensors=(), source='drone'):
        """Returns (zone, newly_confirmed)."""
        radius = min(radius, self.max_radius)
        for z in self.zones:
            if z['type'] != kind:
                continue
            if math.hypot(z['e'] - e, z['n'] - n) < max(MERGE_M.get(kind, 3.0), z['radius'] + radius):
                w = z['hits']
                z['e'] = (z['e'] * w + e) / (w + 1)
                z['n'] = (z['n'] * w + n) / (w + 1)
                # a partial view underestimates size; keep the largest
                z['radius'] = max(z['radius'], radius)
                z['hits'] += 1
                z['last_seen'] = t
                if peak_k is not None:
                    hotter = kind != 'flood'
                    old = z.get('peak_k')
                    z['peak_k'] = peak_k if old is None else (max(old, peak_k) if hotter else min(old, peak_k))
                if conf is not None:
                    z['conf'] = max(z.get('conf') or 0.0, conf)
                z['sensors'] = sorted(set(z['sensors']) | set(sensors))
                newly = z['hits'] == self.confirm_hits
                return z, newly
        z = {
            'id': f'H{len(self.zones) + 1}', 'type': kind, 'label': HAZARD_TYPES[kind][0],
            'e': e, 'n': n, 'radius': radius, 'hits': 1, 'first_seen': t, 'last_seen': t,
            'peak_k': peak_k, 'conf': conf, 'sensors': sorted(sensors), 'source': source,
        }
        self.zones.append(z)
        return z, self.confirm_hits <= 1

    def add_known(self, kind, e, n, radius, label=None):
        """Pre-mission intel (e.g. a collapsed building): confirmed as given."""
        z, _ = self.add(kind, e, n, radius, 0.0, source='intel')
        z['hits'] = max(z['hits'], self.confirm_hits)
        if label:
            z['label'] = label
        return z

    def confirmed(self):
        return [z for z in self.zones if z['hits'] >= self.confirm_hits]


def safe_route(start, goal, hazards, bounds, res=None, margin=3.0):
    """A* ground route from start to goal around hazard zones.

    hazards: zones from HazardMap.confirmed(). Each blocks a disc of its
    radius plus its type's safety buffer; within `margin` m beyond that
    the route may pass but at extra cost, so it keeps its distance where
    it can. bounds: (e_min, n_min, e_max, n_max). Returns
    (points, length_m), or (None, None) if every way is blocked.
    """
    e_min, n_min, e_max, n_max = bounds
    if res is None:
        # 1 m in a small area; coarser for large ones so A* stays ~10k cells
        res = max(1.0, round(max(e_max - e_min, n_max - n_min) / 110.0, 1))
    nx = int(math.ceil((e_max - e_min) / res)) + 1
    ny = int(math.ceil((n_max - n_min) / res)) + 1
    es = e_min + np.arange(nx) * res
    ns = n_min + np.arange(ny) * res
    ee, nn = np.meshgrid(es, ns)
    blocked = np.zeros(ee.shape, dtype=bool)
    penalty = np.zeros(ee.shape)
    for z in hazards:
        d = np.hypot(ee - z['e'], nn - z['n']) - z['radius'] - HAZARD_TYPES[z['type']][1]
        blocked |= d < 0
        penalty += np.where((d >= 0) & (d < margin), (margin - d) / margin * 3.0, 0.0)

    def cell(p):
        return (min(max(int(round((p[1] - n_min) / res)), 0), ny - 1),
                min(max(int(round((p[0] - e_min) / res)), 0), nx - 1))

    s, g = cell(start), cell(goal)
    # The team has to reach the casualty even if they lie near a hazard,
    # and the staging point is wherever it is: free both ends.
    for (r, c) in (s, g):
        near = np.hypot(ee - ee[r, c], nn - nn[r, c]) <= 2.5
        blocked &= ~near

    def h(rc):
        return math.hypot(rc[0] - g[0], rc[1] - g[1]) * res

    steps = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    open_heap = [(h(s), 0.0, s)]
    came = {s: None}
    cost = {s: 0.0}
    while open_heap:
        _, c_cost, cur = heapq.heappop(open_heap)
        if cur == g:
            break
        if c_cost > cost[cur]:
            continue
        for dr, dc in steps:
            r, c = cur[0] + dr, cur[1] + dc
            if not (0 <= r < ny and 0 <= c < nx) or blocked[r, c]:
                continue
            new = c_cost + math.hypot(dr, dc) * res * (1.0 + penalty[r, c])
            if new < cost.get((r, c), float('inf')):
                cost[(r, c)] = new
                came[(r, c)] = cur
                heapq.heappush(open_heap, (new + h((r, c)), new, (r, c)))
    if g not in came:
        return None, None

    cells = []
    cur = g
    while cur is not None:
        cells.append(cur)
        cur = came[cur]
    cells.reverse()

    # Keep only turning points.
    pts = [(float(es[c]), float(ns[r])) for r, c in cells]
    pts[0], pts[-1] = tuple(start), tuple(goal)
    simple = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = simple[-1], pts[i], pts[i + 1]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cross) > 1e-6:
            simple.append(b)
    simple.append(pts[-1])
    length = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(simple, simple[1:]))
    return [[round(p[0], 2), round(p[1], 2)] for p in simple], length


# Walking pace for a rescue team over rubble.
TEAM_SPEED_MS = 1.0


def assess_victims(victims, hazards, staging, bounds):
    """Ground route and rescue priority for each casualty, most urgent first.

    Priority is driven by threat: a casualty close to fire or flood water
    is at risk of getting worse fast. Among equal threats, the one the
    team can reach sooner goes first. Mutates and returns `victims`.
    """
    for v in victims:
        route, length = safe_route(staging, (v['e'], v['n']), hazards, bounds)
        v['route'] = route
        v['route_m'] = round(length, 1) if length is not None else None
        v['eta_min'] = round(length / TEAM_SPEED_MS / 60.0, 1) if length is not None else None
        threat = 0.0
        risks = []
        for z in hazards:
            d = max(0.0, math.hypot(z['e'] - v['e'], z['n'] - v['n']) - z['radius'])
            if z['type'] == 'fire' and d < 15.0:
                threat += 40.0 * (1.0 - d / 15.0)
                risks.append(f"fire {d:.0f} m away")
            elif z['type'] == 'flood' and d < 10.0:
                threat += 25.0 * (1.0 - d / 10.0)
                risks.append(f"flood water {d:.0f} m away")
            elif z['type'] == 'structure' and d < 8.0:
                threat += 15.0 * (1.0 - d / 8.0)
                risks.append(f"unstable structure {d:.0f} m away")
        # Triage: a casualty seen moving (crawling, walking) is responsive -
        # like START's "walking wounded" they come after one who isn't moving.
        if v.get('moving'):
            threat -= 10.0
        v['threat'] = round(threat, 1)
        v['risks'] = risks
        if route is None:
            action = 'No safe ground route: hazards block every way in. Consider air lift.'
        else:
            action = f"Send team on the marked safe route: {v['route_m']:.0f} m, about {v['eta_min']:.1f} min on foot."
        v['action'] = action
    ranked = sorted(victims, key=lambda v: (-v['threat'], v['route_m'] if v['route_m'] is not None else 1e9))
    # (a still casualty with no nearby threat: threat 0; a moving one: -10)
    for i, v in enumerate(ranked):
        v['priority'] = i + 1
    return victims
