#!/usr/bin/env python3
"""Moves the heat shapes of walking / crawling casualties with their actors.

Gazebo's thermal system doesn't apply to animated actors, so every casualty
has an invisible static "heat body" at body temperature (see
generate_region.py). For a casualty that moves, this keeps that heat body
on the actor: it follows the world's simulation clock and puts each heat
body where the actor's trajectory has it at that moment (actor scripts
loop from sim time 0 with auto_start), via the world's set_pose service.

Part of the simulated world, not of the drone: it reads the generator's
resq_mavlink/region_actors.json; the drone never does.

    python3 worldgen/heat_follower.py [--rate 10]
"""
import argparse
import json
import math
import os
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACTORS = os.path.join(ROOT, 'resq_mavlink', 'region_actors.json')


def pose_at(mover, t):
    """(x, y, yaw) on the mover's looping trajectory at sim time t."""
    wps = mover['waypoints']
    t = t % mover['period']
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(wps, wps[1:]):
        if t0 <= t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * da
    _, x, y, a = wps[-1]
    return x, y, a


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()
    with open(ACTORS) as f:
        spec = json.load(f)
    movers = spec['movers']
    service = f"/world/{spec['world']}/set_pose"
    if not movers:
        print('heat_follower: no moving casualties in this world, nothing to do', flush=True)
        return

    node = Node()
    sim = {'t': None}
    lock = threading.Lock()

    def on_clock(msg):
        with lock:
            sim['t'] = msg.sim.sec + msg.sim.nsec * 1e-9

    node.subscribe(Clock, '/clock', on_clock)
    print(f'heat_follower: moving {len(movers)} heat bodies ({", ".join(m["model"] for m in movers)}) at {args.rate:.0f} Hz', flush=True)
    failures = 0
    while True:
        time.sleep(1.0 / args.rate)
        with lock:
            t = sim['t']
        if t is None:
            continue
        for m in movers:
            x, y, yaw = pose_at(m, t)
            req = Pose()
            req.name = m['model']
            req.position.x, req.position.y, req.position.z = x, y, m['z']
            req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            ok, rep = node.request(service, req, Pose, Boolean, 500)
            if not (ok and rep.data):
                failures += 1
                if failures in (1, 10, 100):
                    print(f'heat_follower: set_pose failed for {m["model"]} ({failures} so far)', flush=True)


if __name__ == '__main__':
    main()
