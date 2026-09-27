#!/usr/bin/env python3
"""Monte Carlo comparison of the two search planners.

Flies a simple kinematic drone (constant speed, turns instantly) over the
mission.json search area with the bridge's own ProbabilityMap, lawnmower
spacing and detector model (one analysed frame every FRAME_S, detecting a
casualty in the central part of the footprint with probability POD), and
measures how long each planner takes to find one casualty.

The casualty is placed two ways:
  prior    - drawn from the intel prior (debris fields weigh more), i.e.
             the intel is right about where people tend to be trapped
  uniform  - anywhere in the area with equal chance, i.e. the intel is
             worthless; shows what the probability planner costs then

This measures the planners, not the detector or the sim: the detector
model is the same simple POD for both.

    python3 resq_mavlink/search_benchmark.py [--runs 300] [--night]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'resq_mavlink_bridge'))
from resq_mavlink_bridge.search_map import ProbabilityMap, camera_half_extents  # noqa: E402

# Must match mavlink_bridge.py
SPEED = 2.0
HFOV, W, H = 1.2, 640, 480
LANE_OVERLAP = 0.3
POD = {'day': 0.7, 'night': 0.6}
FRAME_S = {'day': 2.0, 'night': 1.0}  # night frames closer than 1 s count once
REPLAN_S, HYSTERESIS, REACHED_M = 1.0, 1.25, 2.0
WAYPOINT_RADIUS_M = 1.5
DT = 0.1
MAX_T = 900.0
EARTH_RADIUS_M = 6371000.0


def load_mission():
    with open(os.path.join(HERE, 'mission.json')) as f:
        mission = json.load(f)
    # Launch point = ArduPilot SITL's default home, the sim's origin.
    home = (-35.363262, 149.165237)
    t = mission['target']
    north = math.radians(t['latitude'] - home[0]) * EARTH_RADIUS_M
    east = math.radians(t['longitude'] - home[1]) * EARTH_RADIUS_M * math.cos(math.radians(home[0]))
    return mission, (east, north)


def lawnmower(center, width, height, alt, start):
    focal = (W / 2.0) / math.tan(HFOV / 2.0)
    spacing = alt * H / focal * (1.0 - LANE_OVERLAP)
    lanes = max(1, math.ceil(width / spacing))
    first, last = -width / 2 + spacing / 2, width / 2 - spacing / 2
    lane_e = [first + i * (last - first) / (lanes - 1) for i in range(lanes)]
    if start[0] > center[0]:
        lane_e.reverse()
    ends = [-height / 2, height / 2]
    if start[1] > center[1]:
        ends.reverse()
    wps = []
    for i, e in enumerate(lane_e):
        a, b = ends if i % 2 == 0 else ends[::-1]
        wps += [(center[0] + e, center[1] + a), (center[0] + e, center[1] + b)]
    return wps


def sees(pos, yaw, alt, victim, shrink=0.8):
    half_f, half_r = camera_half_extents(alt, HFOV, W, H)
    de, dn = victim[0] - pos[0], victim[1] - pos[1]
    f = de * math.sin(yaw) + dn * math.cos(yaw)
    r = de * math.cos(yaw) - dn * math.sin(yaw)
    return abs(f) <= half_f * shrink and abs(r) <= half_r * shrink


def fly(planner, mission, center, victim, rng, mode):
    s = mission['search']
    w, h, alt = s['width_m'], s['height_m'], s['altitude_m']
    pmap = ProbabilityMap(center, w, h, 2.0, zones=mission['prior_zones'])
    pos, yaw = np.array([0.0, 0.0]), 0.0
    wps = lawnmower(center, w, h, alt, pos) if planner == 'grid' else None
    wi, target, last_plan, next_frame = 0, None, -1e9, 0.0
    t = 0.0
    while t < MAX_T:
        if t >= next_frame:
            next_frame = t + FRAME_S[mode]
            if sees(pos, yaw, alt, victim) and rng.random() < POD[mode]:
                return t
            pmap.observe(pos[0], pos[1], alt, yaw, POD[mode], HFOV, W, H)
        if planner == 'grid':
            if wi >= len(wps):
                return None
            goal = np.array(wps[wi])
            if np.hypot(*(goal - pos)) < WAYPOINT_RADIUS_M:
                wi += 1
                continue
        else:
            reached = target is not None and math.hypot(pos[0] - target[0], pos[1] - target[1]) < REACHED_M
            if target is None or reached or t - last_plan >= REPLAN_S:
                last_plan = t
                best, score = pmap.best_target(pos[0], pos[1], SPEED)
                if target is None or reached or score > pmap.score_at(target, pos[0], pos[1], SPEED) * HYSTERESIS:
                    target = best
            goal = np.array(target)
        d = goal - pos
        dist = float(np.hypot(*d))
        if dist > 1e-6:
            yaw = math.atan2(d[0], d[1])
            pos = pos + d / dist * min(dist, SPEED * DT)
        t += DT
    return None


def sample_victim(pmap, how, rng):
    if how == 'uniform':
        return (pmap.e0 + rng.random() * pmap.width, pmap.n0 + rng.random() * pmap.height)
    i = rng.choice(pmap.prior.size, p=pmap.prior.ravel())
    cw, ch = pmap.width / pmap.nx, pmap.height / pmap.ny
    return (pmap.ee.ravel()[i] + (rng.random() - 0.5) * cw, pmap.nn.ravel()[i] + (rng.random() - 0.5) * ch)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', type=int, default=300)
    ap.add_argument('--night', action='store_true', help='thermal detector timing/POD')
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    mode = 'night' if args.night else 'day'
    mission, center = load_mission()
    s = mission['search']
    ref = ProbabilityMap(center, s['width_m'], s['height_m'], 2.0, zones=mission['prior_zones'])

    print(f"{args.runs} runs per case, {mode} detector model (POD {POD[mode]}, a frame every {FRAME_S[mode]} s), "
          f"{s['width_m']:.0f}x{s['height_m']:.0f} m at {SPEED} m/s\n")
    marks = (60, 120, 300)
    print(f"{'casualty placed':<16}{'planner':<9}" + ''.join(f"{f'found <{m}s':>13}" for m in marks) + f"{'median':>9}")
    for how in ('prior', 'uniform'):
        rng = np.random.default_rng(args.seed)
        victims = [sample_victim(ref, how, rng) for _ in range(args.runs)]
        medians = {}
        for planner in ('grid', 'bayes'):
            prng = np.random.default_rng(args.seed + 7)
            times = np.array([fly(planner, mission, center, v, prng, mode) or np.inf for v in victims])
            medians[planner] = float(np.median(times))
            print(f"{how:<16}{planner:<9}" + ''.join(f"{np.mean(times < m):>13.0%}" for m in marks)
                  + f"{medians[planner]:>8.0f}s")
        g, b = medians['grid'], medians['bayes']
        print(f"{'':<25}median time to find: {100 * abs(1 - b / g):.0f}% "
              f"{'faster' if b < g else 'slower'} with the probability planner\n")
    print("grid stops after one pass (as in the mission); unfound runs count as never.")


if __name__ == '__main__':
    main()
