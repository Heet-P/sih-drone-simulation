#!/usr/bin/env python3
"""Offline benchmark of the search in the 200 x 200 m region world.

Casualties come from the real world generator (worldgen/generate_region.py,
one placement per seed); the drone is a kinematic model flying the bridge's
own ProbabilityMap planner. Detector model: a casualty inside the central
part of the footprint is detected with POD per 1 s look (as the bridge
counts looks), then the drone spends VERIFY_S verifying and marking them.

It measures planners and speeds, not the detector or Gazebo (no obstacles:
climbing over forest would slow real flights down a little).

    python3 resq_mavlink/region_benchmark.py [--seeds 40] [--minutes 15]
"""
import argparse
import importlib.util
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'resq_mavlink_bridge'))
from resq_mavlink_bridge.search_map import ProbabilityMap, camera_half_extents  # noqa: E402

spec = importlib.util.spec_from_file_location('gen', os.path.join(ROOT, 'worldgen', 'generate_region.py'))
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

HFOV, W, H, ALT = 1.2, 640, 480, 8.0
POD = 0.7
LOOK_S = 1.0
VERIFY_S = 12.0      # hover to verify + fly to the drop point + mark
REPLAN_S, HYSTERESIS, REACHED_M = 1.0, 1.25, 2.0
DT = 0.2
AREA = 192.0


def sees(pos, yaw, v, shrink=0.8):
    hf, hr = camera_half_extents(ALT, HFOV, W, H)
    de, dn = v[0] - pos[0], v[1] - pos[1]
    f = de * math.sin(yaw) + dn * math.cos(yaw)
    r = de * math.cos(yaw) - dn * math.sin(yaw)
    return abs(f) <= hf * shrink and abs(r) <= hr * shrink


def lawnmower(speed):
    focal = (W / 2.0) / math.tan(HFOV / 2.0)
    spacing = ALT * H / focal * 0.7
    lanes = int(math.ceil(AREA / spacing))
    wps = []
    for i in range(lanes):
        e = -AREA / 2 + spacing / 2 + i * (AREA - spacing) / (lanes - 1)
        a, b = (-AREA / 2, AREA / 2) if i % 2 == 0 else (AREA / 2, -AREA / 2)
        wps += [(e, a), (e, b)]
    return wps


def fly(planner, victims, speed, prior, max_t, rng, region=(0.0, 0.0)):
    pmap = ProbabilityMap((0, 0), AREA, AREA, 2.0, prior_map=prior, region_radius=region[0],
                          region_weight=region[1]) if planner != 'grid' else None
    wps = lawnmower(speed) if planner == 'grid' else None
    pos, yaw = np.array([0.0, 0.0]), 0.0
    wi, target, last_plan, next_look = 0, None, -1e9, 0.0
    found = {}
    t = 0.0
    while t < max_t and len(found) < len(victims):
        if t >= next_look:
            next_look = t + LOOK_S
            hit = None
            for i, v in enumerate(victims):
                if i not in found and sees(pos, yaw, v) and rng.random() < POD:
                    hit = i
                    break
            if hit is not None:
                found[hit] = t
                t += VERIFY_S
                if pmap is not None:
                    pmap.remove_disk(victims[hit][0], victims[hit][1], 4.0)
                    target = None
                continue
            if pmap is not None:
                pmap.observe(pos[0], pos[1], ALT, yaw, POD, HFOV, W, H)
        if planner == 'grid':
            if wi >= len(wps):
                break
            goal = np.array(wps[wi])
            if np.hypot(*(goal - pos)) < 1.5:
                wi += 1
                continue
        else:
            reached = target is not None and math.hypot(pos[0] - target[0], pos[1] - target[1]) < REACHED_M
            if target is None or reached or t - last_plan >= REPLAN_S:
                last_plan = t
                best, score = pmap.best_target(pos[0], pos[1], speed)
                if target is None or reached or score > pmap.score_at(target, pos[0], pos[1], speed) * HYSTERESIS:
                    target = best
            goal = np.array(target)
        d = goal - pos
        dist = float(np.hypot(*d))
        if dist > 1e-6:
            yaw = math.atan2(d[0], d[1])
            pos = pos + d / dist * min(dist, speed * DT)
        t += DT
    return sorted(found.values())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seeds', type=int, default=40)
    ap.add_argument('--minutes', type=float, default=15)
    ap.add_argument('--speeds', default='5')
    ap.add_argument('--region', default='15:0.5,15:1,25:0.5', help='regional term radius:weight list')
    args = ap.parse_args()
    L = gen.build_layout(random.Random(gen.SCENERY_SEED))
    prior_grid = np.load(os.path.join(HERE, 'prior_region.npy'))
    prior = (prior_grid, -96.0, -96.0, 2.0)
    worlds = []
    for seed in range(1, args.seeds + 1):
        cas = gen.place_casualties(L, seed, 6)
        worlds.append([(c['x'], c['y']) for c in cas])
    max_t = args.minutes * 60
    print(f"{args.seeds} seeds x 6 casualties, {args.minutes:.0f} min limit, POD {POD}/look, {VERIFY_S:.0f} s per find\n")
    print(f"{'planner':<22}{'speed':>6}{'found in 5 min':>16}{'10 min':>9}{'15 min':>9}{'median 1st':>12}{'median 3rd':>12}")
    configs = [('grid', 'lawnmower', None, (0, 0)), ('bayes', 'map, land-use prior', prior, (0, 0))]
    for r, w in [(float(x.split(':')[0]), float(x.split(':')[1])) for x in args.region.split(',') if x]:
        configs.append(('bayes', f'  + regional {r:.0f} m x{w:g}', prior, (r, w)))
    for speed in [float(s) for s in args.speeds.split(',')]:
        for planner, label, pr, region in configs:
            rng = random.Random(99)
            times = [fly(planner, w, speed, pr, max_t, rng, region) for w in worlds]
            def frac(limit):
                return np.mean([sum(1 for x in ts if x <= limit) / 6 for ts in times])
            def med(k):
                vals = [ts[k - 1] if len(ts) >= k else math.inf for ts in times]
                m = float(np.median(vals))
                return f"{m:.0f} s" if math.isfinite(m) else "never"
            print(f"{label:<22}{speed:>5.0f} {frac(300):>15.0%}{frac(600):>9.0%}{frac(900):>9.0%}{med(1):>12}{med(3):>12}")
        print()


if __name__ == '__main__':
    main()
