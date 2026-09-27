#!/usr/bin/env python3
"""Generate the 200 x 200 m "region" world: city, forest, village, river.

    python3 worldgen/generate_region.py                # scenery + world, casualty seed 1
    python3 worldgen/generate_region.py --seed 42      # new casualty placement only
    python3 worldgen/generate_region.py --scenery      # force-rebuild the scenery too

Built for frame rate from the start (see the README's performance notes):
- The whole landscape is ONE static model (region_scenery) made of a few
  merged meshes: every tree in one OBJ, every procedural building in
  another, roads/fields/ground in a third. Hundreds of separate models
  would each cost a draw call in the Gazebo window, both drone cameras
  and the lidar; merged, it's a handful per material.
- No collision geometry on scenery: the drone never touches it (the lidar
  and cameras work from rendering), so physics is as cheap as in the
  small world. Only the ground plane collides.
- A few detailed "hero" models from Gazebo Fuel (render-checked in
  Harmonic: apartment, office, post office, Indian house, collapsed
  house, bus, cars, truss bridge) for close-up realism.

Seed-dependent: only the casualties. They are placed on open ground the
camera can see (street, rubble, village lane, field, clearing, riverbank),
never under tree crowns, inside buildings or in water. Placement follows
the kind of places people are found after a disaster, which is also what
the mission intel's prior says - see the README for why that is fair and
what the benchmark does when the prior is wrong.

Outputs:
  ardu_ws/src/ardupilot_gazebo/models/region_scenery/   static scenery model
  ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/region.sdf  (+ install copy)
  resq_mavlink/mission_region.json    intel for the mission bridge (no casualty positions)
  resq_mavlink/region_truth.json      ground truth, for scoring test runs only
  command_center/frontend/basemaps/region.png   pre-disaster basemap for the dashboard
"""
import argparse
import json
import math
import os
import random
import shutil

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(ROOT, 'ardu_ws/src/ardupilot_gazebo/models')
SCENERY = os.path.join(MODELS, 'region_scenery')
WORLDS_SRC = os.path.join(ROOT, 'ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds')
WORLDS_INSTALL = os.path.join(ROOT, 'ardu_ws/install/ardupilot_gz_gazebo/share/ardupilot_gz_gazebo/worlds')
MISSION = os.path.join(ROOT, 'resq_mavlink/mission_region.json')
TRUTH = os.path.join(ROOT, 'resq_mavlink/region_truth.json')
ACTORS = os.path.join(ROOT, 'resq_mavlink/region_actors.json')  # moving heat shapes, for heat_follower.py
BASEMAP = os.path.join(ROOT, 'command_center/frontend/basemaps/region.png')

HALF = 100.0          # world is -100..100 m in east (x) and north (y)
SCENERY_SEED = 7      # fixed: the landscape is the same every run
HOME_LAT, HOME_LON = -35.3632621, 149.1652374  # = disaster.sdf, SITL home


# ---------------------------------------------------------------------------
# Tiny OBJ writer (Z up, flat normals, per-material face groups)
# ---------------------------------------------------------------------------

class Mesh:
    def __init__(self):
        self.v, self.vt, self.vn = [], [], []
        self.faces = {}  # material -> [[(vi, ti, ni), ...], ...]

    def _v(self, p):
        self.v.append(p)
        return len(self.v)

    def _t(self, uv):
        self.vt.append(uv)
        return len(self.vt)

    def poly(self, mat, pts, uvs, outward=None):
        """Planar polygon (CCW seen from its front). outward: a point the
        normal must face away from (e.g. a box centre), to fix winding."""
        p = np.array(pts, float)
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if outward is not None and np.dot(n, p.mean(axis=0) - np.array(outward, float)) < 0:
            pts, uvs, n = pts[::-1], uvs[::-1], -n
        norm = np.linalg.norm(n)
        n = n / norm if norm > 0 else np.array([0, 0, 1.0])
        self.vn.append(tuple(n))
        ni = len(self.vn)
        self.faces.setdefault(mat, []).append([(self._v(tuple(q)), self._t(tuple(t)), ni) for q, t in zip(pts, uvs)])

    def box(self, cx, cy, z0, sx, sy, sz, yaw, mat_side, mat_top, uv_m=(4.0, 3.0), top_uv=10.0):
        c, s = math.cos(yaw), math.sin(yaw)
        def P(x, y, z):
            return (cx + c * x - s * y, cy + s * x + c * y, z)
        hx, hy = sx / 2, sy / 2
        centre = (cx, cy, z0 + sz / 2)
        corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        for i in range(4):
            (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % 4]
            L = math.hypot(x1 - x0, y1 - y0)
            u1, v1 = L / uv_m[0], sz / uv_m[1]
            self.poly(mat_side, [P(x0, y0, z0), P(x1, y1, z0), P(x1, y1, z0 + sz), P(x0, y0, z0 + sz)],
                      [(0, 0), (u1, 0), (u1, v1), (0, v1)], outward=centre)
        top = [P(x, y, z0 + sz) for x, y in corners]
        self.poly(mat_top, top, [(q[0] / top_uv, q[1] / top_uv) for q in top], outward=centre)

    def gable_roof(self, cx, cy, z0, sx, sy, h, yaw, mat):
        """Pitched roof over an sx (along ridge) x sy footprint."""
        c, s = math.cos(yaw), math.sin(yaw)
        def P(x, y, z):
            return (cx + c * x - s * y, cy + s * x + c * y, z)
        hx, hy, e = sx / 2 + 0.3, sy / 2 + 0.4, 0.0
        a, b = P(-hx, -hy, z0), P(hx, -hy, z0)
        cc, d = P(hx, hy, z0), P(-hx, hy, z0)
        r0, r1 = P(-hx, 0, z0 + h), P(hx, 0, z0 + h)
        centre = (cx, cy, z0 + h / 3)
        slope = math.hypot(hy, h)
        self.poly(mat, [a, b, r1, r0], [(0, 0), (sx / 2, 0), (sx / 2, slope / 2), (0, slope / 2)], outward=centre)
        self.poly(mat, [cc, d, r0, r1], [(0, 0), (sx / 2, 0), (sx / 2, slope / 2), (0, slope / 2)], outward=centre)
        self.poly(mat, [b, cc, r1], [(0, 0), (1, 0), (0.5, 0.5)], outward=centre)
        self.poly(mat, [d, a, r0], [(0, 0), (1, 0), (0.5, 0.5)], outward=centre)
        _ = e

    def cylinder(self, cx, cy, z0, r, h, mat, sides=6):
        centre = (cx, cy, z0 + h / 2)
        for i in range(sides):
            a0, a1 = 2 * math.pi * i / sides, 2 * math.pi * (i + 1) / sides
            p0 = (cx + r * math.cos(a0), cy + r * math.sin(a0))
            p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1))
            self.poly(mat, [(*p0, z0), (*p1, z0), (*p1, z0 + h), (*p0, z0 + h)],
                      [(0, 0), (1, 0), (1, h), (0, h)], outward=centre)

    def cone(self, cx, cy, z0, r, h, mat, sides=8):
        apex = (cx, cy, z0 + h)
        centre = (cx, cy, z0 + h / 3)
        for i in range(sides):
            a0, a1 = 2 * math.pi * i / sides, 2 * math.pi * (i + 1) / sides
            p0 = (cx + r * math.cos(a0), cy + r * math.sin(a0), z0)
            p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1), z0)
            self.poly(mat, [p0, p1, apex], [(0, 0), (1, 0), (0.5, 1)], outward=centre)
            self.poly(mat, [p1, p0, (cx, cy, z0)], [(1, 0), (0, 0), (0.5, 0.5)], outward=(cx, cy, z0 + 1))

    def blob(self, cx, cy, zc, rx, rz, mat, rng):
        """Low-poly broadleaf crown: a jittered octahedron subdivided once."""
        verts = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        tris = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]
        cache = {}
        def mid(i, j):
            k = tuple(sorted((i, j)))
            if k not in cache:
                m = np.add(verts[i], verts[j]) / 2
                verts.append(tuple(m / np.linalg.norm(m)))
                cache[k] = len(verts) - 1
            return cache[k]
        sub = []
        for a, b, c in tris:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            sub += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        jit = [1 + rng.uniform(-0.15, 0.15) for _ in verts]
        P = [(cx + rx * v[0] * j, cy + rx * v[1] * j, zc + rz * v[2] * j) for v, j in zip(verts, jit)]
        for a, b, c in sub:
            self.poly(mat, [P[a], P[b], P[c]], [(0, 0), (1, 0), (0.5, 1)], outward=(cx, cy, zc))

    def write(self, path, mtl_file):
        with open(path, 'w') as f:
            f.write(f'# generated by worldgen/generate_region.py\nmtllib {mtl_file}\n')
            for p in self.v:
                f.write(f'v {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n')
            for t in self.vt:
                f.write(f'vt {t[0]:.4f} {t[1]:.4f}\n')
            for n in self.vn:
                f.write(f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n')
            for mat, faces in self.faces.items():
                f.write(f'usemtl {mat}\n')
                for face in faces:
                    f.write('f ' + ' '.join(f'{a}/{b}/{c}' for a, b, c in face) + '\n')

    def tri_count(self):
        return sum(len(f) - 2 for fs in self.faces.values() for f in fs)


def write_mtl(path, materials):
    with open(path, 'w') as f:
        for name, m in materials.items():
            f.write(f'newmtl {name}\nKa {m[0]} {m[1]} {m[2]}\nKd {m[0]} {m[1]} {m[2]}\nKs 0.05 0.05 0.05\nNs 10\nd 1\nillum 2\n')
            if len(m) > 3:
                f.write(f'map_Kd {m[3]}\n')
            f.write('\n')


# ---------------------------------------------------------------------------
# Textures (generated, so the repo has no third-party texture licences)
# ---------------------------------------------------------------------------

def noise(size, scale, rng):
    """Smooth noise in 0..1 that TILES seamlessly (low-pass filtered white
    noise via FFT is periodic), so repeated ground textures show no seams."""
    white = rng.random((size, size))
    f = np.fft.fftfreq(size)
    k = np.sqrt(f[:, None] ** 2 + f[None, :] ** 2)
    smooth = np.real(np.fft.ifft2(np.fft.fft2(white) * np.exp(-(k * scale * 2.2) ** 2)))
    smooth -= smooth.min()
    return (smooth / (smooth.max() or 1)).astype(np.float32)


def make_textures(out, rng):
    S = 512
    # Grass / dry earth
    n1, n2 = noise(S, 32, rng), noise(S, 8, rng)
    g = np.stack([60 + 40 * n1 + 20 * n2, 95 + 45 * n1 + 20 * n2, 70 + 30 * n1], axis=2)  # BGR
    dirt = (noise(S, 64, rng) > 0.62)[..., None]
    g = np.where(dirt, np.stack([80 + 30 * n2, 105 + 30 * n2, 125 + 30 * n2], axis=2), g)
    cv2.imwrite(os.path.join(out, 'grass.png'), np.clip(g, 0, 255).astype(np.uint8))
    # Asphalt
    a = 70 + 25 * noise(S, 4, rng) + 10 * noise(S, 32, rng)
    cv2.imwrite(os.path.join(out, 'asphalt.png'), np.clip(np.stack([a, a, a + 3], axis=2), 0, 255).astype(np.uint8))
    # Farmland rows
    rows = (np.sin(np.arange(S) / S * 2 * math.pi * 16) > 0).astype(np.float32)
    f = np.stack([60 + 30 * rows[None, :] + 20 * n2, 110 + 50 * rows[None, :] + 20 * n2,
                  120 + 30 * rows[None, :] + 0 * n2], axis=2)
    cv2.imwrite(os.path.join(out, 'farmland.png'), np.clip(f, 0, 255).astype(np.uint8))
    # Facade: one 4 m x 3 m floor bay with a window
    fac = np.full((256, 256, 3), (150, 165, 175), np.float32) + 20 * noise(256, 16, rng)[..., None]
    cv2.rectangle(fac, (60, 70), (196, 200), (60, 55, 50), -1)
    cv2.rectangle(fac, (68, 78), (188, 192), (120, 100, 80), -1)
    cv2.line(fac, (128, 78), (128, 192), (60, 55, 50), 6)
    cv2.imwrite(os.path.join(out, 'facade.png'), np.clip(fac, 0, 255).astype(np.uint8))
    # Mud-plaster hut wall and thatch roof
    mud = np.stack([95 + 30 * n2, 140 + 30 * n2, 175 + 30 * n2], axis=2)
    cv2.imwrite(os.path.join(out, 'mud.png'), np.clip(mud, 0, 255).astype(np.uint8))
    th = 90 + 60 * (noise(S, 2, rng) * 0.6 + 0.4 * (np.sin(np.arange(S)[:, None] / 3.0) > 0))
    cv2.imwrite(os.path.join(out, 'thatch.png'), np.clip(np.stack([th * 0.45, th * 0.8, th], axis=2), 0, 255).astype(np.uint8))
    # Packed dirt (forest trail)
    d = 110 + 40 * noise(S, 8, rng) + 25 * noise(S, 48, rng)
    cv2.imwrite(os.path.join(out, 'dirt.png'), np.clip(np.stack([d * 0.62, d * 0.82, d], axis=2), 0, 255).astype(np.uint8))
    # Concrete (rubble, pavement)
    c = 140 + 40 * noise(S, 8, rng) + 20 * noise(S, 64, rng)
    cv2.imwrite(os.path.join(out, 'concrete.png'), np.clip(np.stack([c, c, c * 1.02], axis=2), 0, 255).astype(np.uint8))


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

def in_poly(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def river_centre(n):
    """River centreline (east as a function of north) in the south-east."""
    return 62 + 10 * math.sin((n + 100) / 23.0) + 0.12 * (n + 60)


def build_layout(rng):
    L = {'roads': [], 'buildings': [], 'huts': [], 'trees': [], 'fields': [], 'hero': [], 'rubble': [],
         'slabs': [], 'water': [], 'flood': [], 'fires': [], 'open': [], 'structures': []}
    # Main roads through the launch area, plus the bridge road in the south-east
    L['roads'] += [(-100, -4, 100, 4), (-4, -100, 4, 100), (4, -64, 100, -56)]
    # City streets (north-east)
    for e in (20, 50, 80):
        L['roads'].append((e - 3, 16, e + 3, 100))
    for n in (20, 50, 80):
        L['roads'].append((16, n - 3, 100, n + 3))
    # City blocks between streets: buildings, some collapsed
    heroes = {(35, 65): ('Apartment', 'OpenRobotics/Apartment', 0.0),
              (65, 35): ('Office Building', 'OpenRobotics/Office Building', 1.57),
              (65, 65): ('Post office', 'OpenRobotics/Post office', 0.0)}
    for bx in (35, 65, 92):
        for by in (35, 65, 92):
            if (bx, by) in heroes:
                label, uri, yaw = heroes[(bx, by)]
                L['hero'].append({'name': label.lower().replace(' ', '_'), 'uri': uri, 'x': bx, 'y': by, 'yaw': yaw})
                L['structures'].append({'label': label + ' (standing)', 'e': bx, 'n': by, 'r': 11, 'collapsed': False})
                continue
            w = 16 if bx < 90 else 10
            collapsed = rng.random() < 0.4
            for k in range(2):
                ox = (k - 0.5) * w * 0.55
                sx, sy = w * 0.45, w * 0.8
                cx, cy = bx + ox, by
                if collapsed and k == 0:
                    # Collapsed: a low rubble heap and tilted slabs where the building was
                    L['rubble'].append((cx, cy, sx * 1.1, sy * 0.9, rng.uniform(1.6, 2.6)))
                    L['slabs'].append((cx + 1, cy - 2, 3.0, 2.2, 0.25, rng.uniform(-0.5, 0.5)))
                    L['structures'].append({'label': 'Collapsed building', 'e': cx, 'n': cy, 'r': max(sx, sy) / 2 + 2, 'collapsed': True})
                    L['open'].append(('rubble', cx + sx * 0.7, cy + rng.uniform(-3, 3)))
                    L['open'].append(('rubble', cx - sx * 0.7, cy + rng.uniform(-3, 3)))
                else:
                    h = rng.choice([6, 9, 12, 15, 18, 21])
                    L['buildings'].append((cx, cy, sx, sy, h, 0.0))
                    L['structures'].append({'label': f'Building ({h} m)', 'e': cx, 'n': cy, 'r': max(sx, sy) / 2 + 1, 'collapsed': False})
            for _ in range(3):
                L['open'].append(('street', bx + rng.uniform(-13, 13), by - 15 + rng.uniform(-1.2, 1.2)))
    # A burning collapsed block with smoke, and a smouldering debris pile
    L['fires'].append({'name': 'fire_city', 'x': 50, 'y': 92, 'smoke': True})
    L['fires'].append({'name': 'fire_debris', 'x': 28, 'y': 45, 'smoke': False})

    # Forest (north-west) with clearings
    clearings = [(-60, 60, 9), (-35, 82, 7), (-80, 30, 8)]
    L['clearings'] = clearings
    for c in clearings:
        for _ in range(3):
            a, r = rng.uniform(0, 2 * math.pi), rng.uniform(0, c[2] - 3)
            L['open'].append(('clearing', c[0] + r * math.cos(a), c[1] + r * math.sin(a)))
    y = 18
    while y < 98:
        x = -98
        while x < -8:
            px, py = x + rng.uniform(-2.5, 2.5), y + rng.uniform(-2.5, 2.5)
            if not any(math.hypot(px - c[0], py - c[1]) < c[2] for c in clearings) and px < -8:
                kind = 'pine' if rng.random() < 0.55 else 'broad'
                L['trees'].append((px, py, rng.uniform(6.5, 12.0), kind))
            x += rng.uniform(5.5, 7.5)
        y += rng.uniform(5.5, 7.5)
    # A forest trail (dirt)
    L['trails'] = [(-60, 40, -56, 96)]

    # Village (south-west): huts along lanes, fields around
    L['roads'] += [(-100, -42, -8, -38), (-62, -100, -58, -4)]
    heroes_v = [(-30, -25, 0.3), (-80, -25, -0.4), (-40, -75, 1.2)]
    for i, (hx, hy, yaw) in enumerate(heroes_v):
        L['hero'].append({'name': f'indian_house_{i}', 'uri': 'hisuki/Indian House', 'x': hx, 'y': hy, 'yaw': yaw})
        L['structures'].append({'label': 'Village house', 'e': hx, 'n': hy, 'r': 6, 'collapsed': False})
    L['hero'].append({'name': 'collapsed_house_village', 'uri': 'OpenRobotics/Collapsed House', 'x': -45, 'y': -52, 'yaw': 0.6})
    L['structures'].append({'label': 'Collapsed house', 'e': -45, 'n': -52, 'r': 8, 'collapsed': True})
    L['open'] += [('rubble', -39, -47), ('rubble', -52, -57)]
    for _ in range(26):
        for _ in range(40):
            hx, hy = rng.uniform(-96, -14), rng.uniform(-96, -14)
            ok = all(math.hypot(hx - h[0], hy - h[1]) > 9 for h in L['huts']) and \
                all(math.hypot(hx - h['x'], hy - h['y']) > 11 for h in L['hero']) and \
                abs(hy + 40) > 6 and abs(hx + 60) > 6
            if ok:
                collapsed = rng.random() < 0.25
                L['huts'].append((hx, hy, rng.uniform(4, 6), rng.uniform(3.5, 5), rng.uniform(0, math.pi), collapsed))
                L['structures'].append({'label': 'Collapsed hut' if collapsed else 'Hut', 'e': hx, 'n': hy, 'r': 4, 'collapsed': collapsed})
                if collapsed:
                    L['open'].append(('rubble', hx + 4, hy))
                break
    for fx, fy, sx, sy in ((-90, -60, 22, 26), (-30, -90, 26, 16), (-88, -90, 18, 14), (-20, -60, 14, 18)):
        L['fields'].append((fx - sx / 2, fy - sy / 2, fx + sx / 2, fy + sy / 2))
        for _ in range(2):
            L['open'].append(('field', fx + rng.uniform(-sx / 3, sx / 3), fy + rng.uniform(-sy / 3, sy / 3)))
    for _ in range(4):
        L['open'].append(('lane', rng.uniform(-90, -20), -40 + rng.uniform(-1, 1)))

    # River (south-east), flood where the west bank overflowed
    pts_w, pts_e = [], []
    for n in np.arange(-100, -6, 4.0):
        c = river_centre(n)
        pts_w.append((c - 5, n))
        pts_e.append((c + 5, n))
    river = pts_w + pts_e[::-1]
    L['water'].append(river)
    for n0, ln, wd in ((-88, 20, 13), (-40, 18, 10), (-20, 12, 9)):
        c = river_centre(n0 + ln / 2) - 5
        L['flood'].append([(c - wd, n0), (c + 1, n0), (c + 1, n0 + ln), (c - wd + 3, n0 + ln)])
        L['open'].append(('riverbank', c - wd - 2.5, n0 + ln / 2))
    L['hero'].append({'name': 'truss_bridge', 'uri': 'OpenRobotics/Truss bridge', 'x': river_centre(-60), 'y': -60, 'yaw': 0.0})
    L['hero'].append({'name': 'bus_stranded', 'uri': 'OpenRobotics/Bus', 'x': river_centre(-80) - 12, 'y': -80, 'yaw': 1.3})
    L['hero'].append({'name': 'house_river', 'uri': 'hisuki/House', 'x': 30, 'y': -30, 'yaw': 0.2})
    L['structures'].append({'label': 'Riverside house', 'e': 30, 'n': -30, 'r': 6, 'collapsed': False})
    L['hero'].append({'name': 'collapsed_house_river', 'uri': 'OpenRobotics/Collapsed House', 'x': 28, 'y': -85, 'yaw': -0.4})
    L['structures'].append({'label': 'Collapsed house', 'e': 28, 'n': -85, 'r': 8, 'collapsed': True})
    L['open'] += [('rubble', 36, -82), ('lane', 20, -60), ('lane', 40, -60)]
    # Cars along roads
    for i, (x, y, yaw, uri) in enumerate([(25, 1.5, 0, 'OpenRobotics/Prius Hybrid'), (-40, -1.5, 3.14, 'OpenRobotics/Hatchback blue'),
                                          (1.5, 60, 1.57, 'OpenRobotics/Hatchback blue'), (70, 51.5, 0.1, 'OpenRobotics/Prius Hybrid')]):
        L['hero'].append({'name': f'car_{i}', 'uri': uri, 'x': x, 'y': y, 'yaw': yaw})
    return L


def blocked(x, y, L, margin):
    """Open ground a casualty could lie on, visible from above."""
    if max(abs(x), abs(y)) > 96 or math.hypot(x, y) < 22:
        return True
    for (cx, cy, sx, sy, h, yaw) in L['buildings']:
        if abs(x - cx) < sx / 2 + margin and abs(y - cy) < sy / 2 + margin:
            return True
    for (hx, hy, sx, sy, yaw, col) in L['huts']:
        if math.hypot(x - hx, y - hy) < max(sx, sy) / 2 + margin and not col:
            return True
    for h in L['hero']:
        if math.hypot(x - h['x'], y - h['y']) < (12 if h['uri'].endswith(('Apartment', 'Building', 'office', 'bridge')) else 6) + margin:
            return True
    for (tx, ty, th, kind) in L['trees']:
        if math.hypot(x - tx, y - ty) < (2.2 if kind == 'pine' else 3.2) + margin:
            return True
    for poly in L['water'] + L['flood']:
        if in_poly(x, y, poly):
            return True
    for f in L['fires']:
        if math.hypot(x - f['x'], y - f['y']) < 4:
            return True
    return False


# ---------------------------------------------------------------------------
# Scenery model
# ---------------------------------------------------------------------------

def build_scenery(L, rng):
    mesh_dir = os.path.join(SCENERY, 'meshes')
    if os.path.isdir(SCENERY):
        shutil.rmtree(SCENERY)
    os.makedirs(mesh_dir)
    make_textures(mesh_dir, rng)

    # Ground, roads, fields, pavement: one mesh
    g = Mesh()
    G = 300.0
    g.poly('grass', [(-G, -G, 0), (G, -G, 0), (G, G, 0), (-G, G, 0)],
           [(-G / 6, -G / 6), (G / 6, -G / 6), (G / 6, G / 6), (-G / 6, G / 6)])
    for (x0, y0, x1, y1) in L['roads']:
        g.poly('asphalt', [(x0, y0, 0.03), (x1, y0, 0.03), (x1, y1, 0.03), (x0, y1, 0.03)],
               [(x0 / 8, y0 / 8), (x1 / 8, y0 / 8), (x1 / 8, y1 / 8), (x0 / 8, y1 / 8)])
    for (x0, y0, x1, y1) in L.get('trails', []):
        g.poly('dirt', [(x0, y0, 0.03), (x1, y0, 0.03), (x1, y1, 0.03), (x0, y1, 0.03)],
               [(x0 / 6, y0 / 6), (x1 / 6, y0 / 6), (x1 / 6, y1 / 6), (x0 / 6, y1 / 6)])
    for (x0, y0, x1, y1) in L['fields']:
        g.poly('farmland', [(x0, y0, 0.02), (x1, y0, 0.02), (x1, y1, 0.02), (x0, y1, 0.02)],
               [(0, 0), ((x1 - x0) / 12, 0), ((x1 - x0) / 12, (y1 - y0) / 12), (0, (y1 - y0) / 12)])
    g.poly('concrete', [(14, 14, 0.015), (100, 14, 0.015), (100, 100, 0.015), (14, 100, 0.015)],
           [(0, 0), (8, 0), (8, 8), (0, 8)])
    g.poly('concrete', [(-12, -12, 0.035), (12, -12, 0.035), (12, 12, 0.035), (-12, 12, 0.035)],
           [(0, 0), (2, 0), (2, 2), (0, 2)])
    g.write(os.path.join(mesh_dir, 'ground.obj'), 'ground.mtl')
    write_mtl(os.path.join(mesh_dir, 'ground.mtl'), {
        'grass': (1, 1, 1, 'grass.png'), 'asphalt': (1, 1, 1, 'asphalt.png'),
        'farmland': (1, 1, 1, 'farmland.png'), 'concrete': (1, 1, 1, 'concrete.png'), 'dirt': (1, 1, 1, 'dirt.png')})

    # Buildings, huts, rubble, slabs: one mesh
    b = Mesh()
    for (cx, cy, sx, sy, h, yaw) in L['buildings']:
        b.box(cx, cy, 0, sx, sy, h, yaw, 'facade', 'roof', uv_m=(4.0, 3.0))
    for (hx, hy, sx, sy, yaw, collapsed) in L['huts']:
        if collapsed:
            b.box(hx, hy, 0, sx * 0.9, sy * 0.8, 0.9, yaw, 'mud', 'thatch', uv_m=(2.0, 2.0), top_uv=2.0)
            b.gable_roof(hx + 0.8, hy + 0.5, 0.9, sx * 0.7, sy * 0.9, 0.9, yaw + 0.35, 'thatch')
        else:
            b.box(hx, hy, 0, sx, sy, 2.6, yaw, 'mud', 'thatch', uv_m=(2.0, 2.0), top_uv=2.0)
            b.gable_roof(hx, hy, 2.6, sx, sy, 1.6, yaw, 'thatch')
    for (cx, cy, sx, sy, h) in L['rubble']:
        for k in range(5):
            b.box(cx + rng.uniform(-sx / 3, sx / 3), cy + rng.uniform(-sy / 3, sy / 3), 0,
                  sx * rng.uniform(0.3, 0.6), sy * rng.uniform(0.3, 0.6), h * rng.uniform(0.3, 1.0),
                  rng.uniform(0, math.pi), 'rubble', 'rubble', uv_m=(2.0, 2.0), top_uv=2.0)
    for (cx, cy, sx, sy, t, yaw) in L['slabs']:
        b.box(cx, cy, 0.4, sx, sy, t, yaw, 'rubble', 'rubble', uv_m=(2.0, 2.0), top_uv=2.0)
    b.write(os.path.join(mesh_dir, 'buildings.obj'), 'buildings.mtl')
    write_mtl(os.path.join(mesh_dir, 'buildings.mtl'), {
        'facade': (1, 1, 1, 'facade.png'), 'roof': (0.42, 0.42, 0.44), 'mud': (1, 1, 1, 'mud.png'),
        'thatch': (1, 1, 1, 'thatch.png'), 'rubble': (0.85, 0.83, 0.8, 'concrete.png')})

    # Trees: one mesh
    t = Mesh()
    for (tx, ty, th, kind) in L['trees']:
        t.cylinder(tx, ty, 0, 0.22, th * 0.35, 'trunk')
        if kind == 'pine':
            t.cone(tx, ty, th * 0.25, th * 0.2, th * 0.75, 'pine')
        else:
            t.blob(tx, ty, th * 0.62, th * 0.28, th * 0.3, 'leaf_a' if rng.random() < 0.5 else 'leaf_b', rng)
    t.write(os.path.join(mesh_dir, 'trees.obj'), 'trees.mtl')
    write_mtl(os.path.join(mesh_dir, 'trees.mtl'), {
        'trunk': (0.33, 0.24, 0.16), 'pine': (0.13, 0.30, 0.16), 'leaf_a': (0.22, 0.42, 0.18), 'leaf_b': (0.30, 0.46, 0.20)})

    stats = {'ground_tris': g.tri_count(), 'building_tris': b.tri_count(), 'tree_tris': t.tri_count(),
             'trees': len(L['trees']), 'buildings': len(L['buildings']), 'huts': len(L['huts'])}

    visuals = []
    for name in ('ground', 'buildings', 'trees'):
        visuals.append(f'''      <visual name="{name}">
        <geometry><mesh><uri>model://region_scenery/meshes/{name}.obj</uri></mesh></geometry>
      </visual>''')
    # Water: thermal cold (river 281 K; flood 279 K, as in disaster.sdf).
    for i, poly in enumerate(L['water'] + L['flood']):
        is_flood = i >= len(L['water'])
        w = Mesh()
        cx, cy = np.mean([p[0] for p in poly]), np.mean([p[1] for p in poly])
        # fan triangulation around the centroid (polygons are near-convex)
        for k in range(len(poly)):
            p0, p1 = poly[k], poly[(k + 1) % len(poly)]
            w.poly('water', [(cx, cy, 0.05), (p0[0], p0[1], 0.05), (p1[0], p1[1], 0.05)],
                   [(0, 0), (1, 0), (0, 1)], outward=(cx, cy, -1))
        fname = f'{"flood" if is_flood else "river"}_{i}.obj'
        w.write(os.path.join(mesh_dir, fname), 'water.mtl')
        temp = 279.0 if is_flood else 281.0
        visuals.append(f'''      <visual name="{fname[:-4]}">
        <geometry><mesh><uri>model://region_scenery/meshes/{fname}</uri></mesh></geometry>
        <material><ambient>0.10 0.13 0.13 1</ambient><diffuse>0.16 0.20 0.19 1</diffuse><specular>0.7 0.7 0.7 1</specular>
          <pbr><metal><roughness>0.08</roughness><metalness>0.0</metalness></metal></pbr></material>
        <cast_shadows>false</cast_shadows>
        <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>{temp}</temperature></plugin>
      </visual>''')
    write_mtl(os.path.join(mesh_dir, 'water.mtl'), {'water': (0.16, 0.20, 0.19)})

    with open(os.path.join(SCENERY, 'model.sdf'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<!-- Generated by worldgen/generate_region.py - edit the generator, not this file.
     Static landscape of the region world as a handful of merged meshes
     ({stats}). No collision except the ground plane. -->
<sdf version="1.9">
  <model name="region_scenery">
    <static>true</static>
    <link name="link">
      <collision name="ground_collision">
        <geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
      </collision>
{chr(10).join(visuals)}
    </link>
  </model>
</sdf>
''')
    with open(os.path.join(SCENERY, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model><name>region_scenery</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf><description>Generated landscape for the region world.</description></model>\n')
    return stats


def build_basemap(L):
    """Pre-disaster top-down map for the dashboard (1 px = 0.25 m)."""
    px = 4
    S = int(2 * HALF * px)
    img = np.full((S, S, 3), (96, 132, 108), np.uint8)
    def P(x, y):
        return int((x + HALF) * px), int((HALF - y) * px)
    for (x0, y0, x1, y1) in L['fields']:
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (92, 160, 170), -1)
    cv2.rectangle(img, P(14, 100), P(100, 14), (150, 150, 150), -1)
    for (x0, y0, x1, y1) in L['roads']:
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (70, 70, 72), -1)
    for (x0, y0, x1, y1) in L.get('trails', []):
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (90, 125, 150), -1)
    for poly in L['water']:
        cv2.fillPoly(img, [np.array([P(*p) for p in poly], np.int32)], (170, 120, 60))
    for (tx, ty, th, kind) in L['trees']:
        cv2.circle(img, P(tx, ty), int((2.0 if kind == 'pine' else 3.0) * px), (50, 100, 52), -1)
    for (cx, cy, sx, sy, h, yaw) in L['buildings']:
        cv2.rectangle(img, P(cx - sx / 2, cy + sy / 2), P(cx + sx / 2, cy - sy / 2), (190, 186, 180), -1)
    for s in L['structures']:
        if s['label'].startswith('Collapsed'):
            cv2.circle(img, P(s['e'], s['n']), int(s['r'] * px * 0.7), (150, 160, 175), -1)
    for (hx, hy, sx, sy, yaw, col) in L['huts']:
        box = cv2.boxPoints(((P(hx, hy)), (sx * px, sy * px), -math.degrees(yaw)))
        cv2.fillPoly(img, [box.astype(np.int32)], (110, 150, 185))
    for h in L['hero']:
        if not h['name'].startswith('car'):
            cv2.circle(img, P(h['x'], h['y']), 6 * px, (190, 186, 180), -1)
    # The basemap is PRE-disaster: flood water, fires and collapses are not on it
    # (collapsed buildings are drawn as their original footprint above).
    os.makedirs(os.path.dirname(BASEMAP), exist_ok=True)
    cv2.imwrite(BASEMAP, img)


PRIOR = os.path.join(ROOT, 'resq_mavlink/prior_region.npy')
PRIOR_CELL = 2.0
PRIOR_HALF = 96.0


def build_prior(L):
    """Where people are likely to be, from the PRE-DISASTER map only (never
    from casualty positions): the way SAR planners build a probability map
    from population and land-use layers.

    around homes/buildings   1.0   people live and work there
    around collapsed ones    3.0   trapped / injured
    roads, lanes, streets    1.0   evacuation routes
    riverbank band           1.2   flood victims reach the bank
    forest clearings, trail  1.2   people shelter in the open
    fields                   0.6
    open ground elsewhere    0.15
    under tree crowns        0.02  invisible from the air anyway
    water, building roofs    0     (not searchable from above)
    """
    n = int(2 * PRIOR_HALF / PRIOR_CELL)
    c = -PRIOR_HALF + (np.arange(n) + 0.5) * PRIOR_CELL
    ee, nn = np.meshgrid(c, c)  # row = north
    prior = np.full(ee.shape, 0.15)
    def near_disc(x, y, r):
        return (ee - x) ** 2 + (nn - y) ** 2 <= r * r
    for (x0, y0, x1, y1) in L['fields']:
        prior[(ee >= x0) & (ee <= x1) & (nn >= y0) & (nn <= y1)] = 0.6
    for (x0, y0, x1, y1) in L['roads'] + L.get('trails', []):
        prior[(ee >= x0 - 1) & (ee <= x1 + 1) & (nn >= y0 - 1) & (nn <= y1 + 1)] = 1.0
    for s in L['structures']:
        m = near_disc(s['e'], s['n'], s['r'] + 10)
        prior[m] = np.maximum(prior[m], 3.0 if s['collapsed'] else 1.0)
    for poly in L['water']:
        for (x, y) in poly:
            m = near_disc(x, y, 12)
            prior[m] = np.maximum(prior[m], 1.2)
    for (x, y, r) in L.get('clearings', []):
        m = near_disc(x, y, r)
        prior[m] = np.maximum(prior[m], 1.2)
    for (x0, y0, x1, y1) in L.get('trails', []):
        prior[(ee >= x0 - 1) & (ee <= x1 + 1) & (nn >= y0) & (nn <= y1)] = 1.2
    for (tx, ty, th, kind) in L['trees']:
        prior[near_disc(tx, ty, 2.2 if kind == 'pine' else 3.2)] = 0.02
    for (cx, cy, sx, sy, h, yaw) in L['buildings']:
        prior[(np.abs(ee - cx) < sx / 2) & (np.abs(nn - cy) < sy / 2)] = 0.0
    for poly in L['water'] + L['flood']:
        inside = np.array([in_poly(x, y, poly) for x, y in zip(ee.ravel(), nn.ravel())]).reshape(ee.shape)
        prior[inside] = 0.0
    prior[(np.abs(ee) < 12) & (np.abs(nn) < 12)] = 0.0  # the command post itself
    np.save(PRIOR, prior.astype(np.float32))
    return prior


# Approximate heights (m) of the Fuel hero models, by model name.
HERO_HEIGHT = {'Apartment': 20.0, 'Office Building': 22.0, 'Post office': 12.0, 'Indian House': 8.0,
               'House': 8.0, 'Collapsed House': 10.0, 'Truss bridge': 8.0, 'Bus': 3.5,
               'Prius Hybrid': 1.6, 'Hatchback blue': 1.6}
HERO_RADIUS = {'Apartment': 12.0, 'Office Building': 12.0, 'Post office': 10.0, 'Indian House': 7.0,
               'House': 7.0, 'Collapsed House': 10.0, 'Truss bridge': 8.0, 'Bus': 6.0,
               'Prius Hybrid': 3.0, 'Hatchback blue': 3.0}


def known_obstacles(L, min_height=4.0):
    """Mapped obstacles taller than min_height: {label, east_m, north_m, radius_m, height_m}."""
    out = []
    for (cx, cy, sx, sy, h, yaw) in L['buildings']:
        out.append({'label': f'Building ({h} m)', 'east_m': round(cx, 1), 'north_m': round(cy, 1),
                    'radius_m': round(math.hypot(sx, sy) / 2, 1), 'height_m': float(h)})
    for hero in L['hero']:
        name = hero['uri'].split('/')[1]
        if HERO_HEIGHT.get(name, 0) > min_height:
            out.append({'label': name, 'east_m': round(hero['x'], 1), 'north_m': round(hero['y'], 1),
                        'radius_m': HERO_RADIUS[name], 'height_m': HERO_HEIGHT[name]})
    return out


def write_mission(L):
    prior = []
    for s in L['structures']:
        if s['collapsed']:
            prior.append({'label': s['label'], 'east_m': round(s['e'], 1), 'north_m': round(s['n'], 1), 'radius_m': 6, 'weight': 3})
    prior += [
        {'label': 'City streets', 'east_m': 58, 'north_m': 58, 'radius_m': 30, 'weight': 0.6},
        {'label': 'Village', 'east_m': -55, 'north_m': -55, 'radius_m': 30, 'weight': 0.6},
        {'label': 'Riverbank (flood risk)', 'east_m': 55, 'north_m': -55, 'radius_m': 22, 'weight': 0.8},
        {'label': 'Forest clearings / trail', 'east_m': -58, 'north_m': 60, 'radius_m': 25, 'weight': 0.3},
    ]
    lat = HOME_LAT
    lon = HOME_LON
    mission = {
        'mission_id': 'RESQ-REGION',
        '_generated': 'worldgen/generate_region.py - pre-disaster intel only, no casualty positions',
        'target': {'latitude': lat, 'longitude': lon, 'altitude': 10},
        # 5 m/s and a 25 min budget (15 min found ~2 of 6 in the real sim): region_benchmark.py measured the land-use
        # map planner finding 43/67/81% of casualties by 5/10/15 min at 5 m/s
        # (lawnmower 31/57/62%); faster flight gained nothing clear.
        'search': {'width_m': 192, 'height_m': 192, 'altitude_m': 8, 'speed_ms': 5.0,
                   'planner': 'bayes', 'on_confirm': 'continue', 'target_pos': 0.85, 'max_search_s': 1500},
        '_local_frame': 'metres east/north of the launch point',
        'staging': {'east_m': -10, 'north_m': -8, 'label': 'Command post / ambulance'},
        'prior_zones': [],
        '_prior_zones_note': 'the prior is prior_map (land-use layers), see build_prior()',
        'prior_map': {'file': 'prior_region.npy', 'e0': -PRIOR_HALF, 'n0': -PRIOR_HALF, 'cell_m': PRIOR_CELL},
        'known_hazards': [{'type': 'structure', 'label': s['label'], 'east_m': round(s['e'], 1), 'north_m': round(s['n'], 1),
                           'radius_m': round(s['r'], 1)} for s in L['structures'] if s['collapsed'] or 'Building (1' in s['label'] or 'Building (2' in s['label']],
        'known_water': [[[round(p[0], 1), round(p[1], 1)] for p in poly] for poly in L['water']],
        # Obstacle heights from the pre-disaster map (as OpenStreetMap
        # building heights would give): the bridge flies over these at
        # height + clearance BEFORE reaching them. The forward lidar alone
        # can't protect a drone moving sideways or braking backwards in a
        # street between tall blocks (two crashes in testing).
        'known_obstacles': known_obstacles(L),
        'basemap': {'url': 'basemaps/region.png', 'e0': -HALF, 'n0': -HALF, 'w': 2 * HALF, 'h': 2 * HALF},
    }
    with open(MISSION, 'w') as f:
        json.dump(mission, f, indent=2)


# ---------------------------------------------------------------------------
# World file with seeded casualties
# ---------------------------------------------------------------------------

CASUALTY_KINDS = ['rubble', 'rubble', 'street', 'lane', 'field', 'clearing', 'riverbank']


def place_casualties(L, seed, count):
    rng = random.Random(seed)
    placed = []
    open_spots = list(L['open'])
    rng.shuffle(open_spots)
    for kind in (rng.choice(CASUALTY_KINDS) for _ in range(count * 6)):
        if len(placed) >= count:
            break
        for (k, x, y) in open_spots:
            if k != kind:
                continue
            x2, y2 = x + rng.uniform(-1.5, 1.5), y + rng.uniform(-1.5, 1.5)
            if blocked(x2, y2, L, 1.0) or any(math.hypot(x2 - p['x'], y2 - p['y']) < 15 for p in placed):
                continue
            placed.append({'id': f'C{len(placed) + 1}', 'x': round(x2, 2), 'y': round(y2, 2), 'yaw': round(rng.uniform(-math.pi, math.pi), 2),
                           'where': kind, 'pose': 'lying'})
            open_spots.remove((k, x, y))
            break
    # Poses from a separate random stream, so a seed's positions don't
    # change when pose options change. Movers need a clear path.
    prng = random.Random(seed * 7919 + 13)
    for c in placed:
        for _ in range(10):
            pose = prng.choices(POSES, POSE_WEIGHTS)[0]
            if pose in MOVES:
                reach = MOVES[pose][0]
                a = c['yaw']
                path = [(c['x'] + t * reach * math.cos(a), c['y'] + t * reach * math.sin(a)) for t in np.linspace(-1, 1, 9)]
                if any(blocked(px, py, L, 0.8) for px, py in path):
                    continue
            c['pose'] = pose
            break
    return placed


# Pose mix; lying / crawling / sitting / waving / walking. Movers pace back
# and forth: (half path length m, speed m/s).
POSES = ['lying', 'crawling', 'sitting', 'waving', 'walking']
POSE_WEIGHTS = [0.35, 0.15, 0.20, 0.15, 0.15]
MOVES = {'walking': (3.0, 1.0), 'crawling': (1.5, 0.3)}
ACTOR_Z = {'lying': 0.15, 'crawling': 0.25, 'sitting': 0.0, 'waving': 1.0, 'walking': 1.0}
ACTOR_PITCH = {'lying': -1.5708, 'crawling': 1.5708}
ACTOR_ANIM = {'lying': 'stand', 'crawling': 'walk', 'sitting': 'sitting', 'waving': 'talk_b', 'walking': 'walk'}


def trajectory(c):
    """Actor waypoints [(time, x, y, yaw)] and loop period; movers pace a
    straight line along their yaw and turn at each end."""
    x, y, a = c['x'], c['y'], c['yaw']
    if c['pose'] not in MOVES:
        return [(0.0, x, y, a), (30.0, x, y, a)], 30.0
    half, v = MOVES[c['pose']]
    dx, dy = half * math.cos(a), half * math.sin(a)
    leg = 2 * half / v
    p0, p1 = (x - dx, y - dy), (x + dx, y + dy)
    wps = [(0.0, *p0, a), (leg, *p1, a), (leg + 0.5, *p1, a + math.pi), (2 * leg + 0.5, *p0, a + math.pi), (2 * leg + 1.0, *p0, a)]
    return wps, 2 * leg + 1.0


HEAT_Z = {'lying': 0.27, 'crawling': 0.27, 'sitting': 0.0, 'waving': 0.0, 'walking': 0.0}
# Heat shapes (model frame, x forward), an adult of ~1.75 m. Skin (head,
# hands) 309 K, clothing 305 K. Lying/crawling: flat along x, head at -x.
HEAT_PARTS = {
    'lying': [('head', '-0.55 0 0 0 0 0', '<sphere><radius>0.123</radius></sphere>', 309),
              ('torso', '-0.15 0 0 0 0 0', '<box><size>0.515 0.381 0.2</size></box>', 305),
              ('pelvis', '0.12 0 0 0 0 0', '<box><size>0.2 0.336 0.18</size></box>', 305),
              ('leg_l', '0.6 0.13 0 0 1.5708 0.12', '<cylinder><radius>0.078</radius><length>0.9</length></cylinder>', 305),
              ('leg_r', '0.6 -0.13 0 0 1.5708 -0.12', '<cylinder><radius>0.078</radius><length>0.9</length></cylinder>', 305),
              ('arm_l', '-0.02 0.24 0 0 1.5708 0', '<cylinder><radius>0.050</radius><length>0.66</length></cylinder>', 305),
              ('arm_r', '-0.02 -0.24 0 0 1.5708 0', '<cylinder><radius>0.050</radius><length>0.66</length></cylinder>', 305),
              ('hand_l', '0.34 0.25 0 0 0 0', '<sphere><radius>0.055</radius></sphere>', 309),
              ('hand_r', '0.34 -0.25 0 0 0 0', '<sphere><radius>0.055</radius></sphere>', 309)],
    'upright': [('head', '0 0 1.62 0 0 0', '<sphere><radius>0.11</radius></sphere>', 309),
                ('torso', '0 0 1.2 0 0 0', '<box><size>0.24 0.4 0.6</size></box>', 305),
                ('hips', '0 0 0.85 0 0 0', '<box><size>0.22 0.34 0.2</size></box>', 305),
                ('leg_l', '0 0.1 0.42 0 0 0', '<cylinder><radius>0.07</radius><length>0.84</length></cylinder>', 305),
                ('leg_r', '0 -0.1 0.42 0 0 0', '<cylinder><radius>0.07</radius><length>0.84</length></cylinder>', 305),
                ('arm_l', '0 0.26 1.15 0 0 0', '<cylinder><radius>0.045</radius><length>0.6</length></cylinder>', 305),
                ('arm_r', '0 -0.26 1.15 0 0 0', '<cylinder><radius>0.045</radius><length>0.6</length></cylinder>', 305)],
    'sitting': [('head', '-0.05 0 0.88 0 0 0', '<sphere><radius>0.11</radius></sphere>', 309),
                ('torso', '-0.05 0 0.55 0 0 0', '<box><size>0.24 0.4 0.55</size></box>', 305),
                ('thigh_l', '0.22 0.1 0.12 0 1.5708 0', '<cylinder><radius>0.08</radius><length>0.45</length></cylinder>', 305),
                ('thigh_r', '0.22 -0.1 0.12 0 1.5708 0', '<cylinder><radius>0.08</radius><length>0.45</length></cylinder>', 305),
                ('shin_l', '0.45 0.1 0.12 0 1.5708 0', '<cylinder><radius>0.06</radius><length>0.4</length></cylinder>', 305),
                ('shin_r', '0.45 -0.1 0.12 0 1.5708 0', '<cylinder><radius>0.06</radius><length>0.4</length></cylinder>', 305),
                ('arm_l', '0.05 0.26 0.5 0 0.4 0', '<cylinder><radius>0.045</radius><length>0.5</length></cylinder>', 305),
                ('arm_r', '0.05 -0.26 0.5 0 0.4 0', '<cylinder><radius>0.045</radius><length>0.5</length></cylinder>', 305)],
}
HEAT_SHAPE = {'lying': 'lying', 'crawling': 'lying', 'sitting': 'sitting', 'waving': 'upright', 'walking': 'upright'}


def heat_yaw(pose, actor_yaw):
    # A face-down (crawling) actor has its head along +x, the face-up
    # lying one along -x, as the lying heat shape has.
    return actor_yaw + math.pi if pose == 'crawling' else actor_yaw


def heat_body(name, pose, x, y, yaw):
    """Body heat matching the actor's pose (the thermal system doesn't apply
    to actors). Moving casualties' heat is moved by worldgen/heat_follower.py."""
    vis = '\n'.join(
        f'        <visual name="{p}"><pose>{pose_}</pose><geometry>{geo}</geometry><transparency>1.0</transparency>'
        f'<cast_shadows>false</cast_shadows><plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">'
        f'<temperature>{t}</temperature></plugin></visual>' for p, pose_, geo, t in HEAT_PARTS[HEAT_SHAPE[pose]])
    return f'''    <model name="{name}_heat"><static>true</static><pose>{x:.2f} {y:.2f} {HEAT_Z[pose]} 0 0 {heat_yaw(pose, yaw):.3f}</pose>
      <link name="body">
{vis}
      </link>
    </model>'''


def actor(name, c):
    base = 'https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes'
    anim = ACTOR_ANIM[c['pose']]
    z, pitch = ACTOR_Z[c['pose']], ACTOR_PITCH.get(c['pose'], 0.0)
    wps, _ = trajectory(c)
    points = ''.join(f'<waypoint><time>{t:.2f}</time><pose>{x:.2f} {y:.2f} {z} 0 {pitch} {yaw:.3f}</pose></waypoint>'
                     for t, x, y, yaw in wps)
    return f'''    <actor name="{name}">
      <skin><filename>{base}/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="{anim}"><filename>{base}/{anim}.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="{anim}">{points}</trajectory>
      </script>
    </actor>'''


def write_world(L, casualties):
    header_src = open(os.path.join(WORLDS_SRC, 'disaster.sdf')).read()
    # Reuse disaster.sdf's proven header: physics, systems, scene, atmosphere
    # (thermal background), lights (sun/fill names used by day/night), sky
    # domes and spherical coordinates. Everything after them is ours.
    start = header_src.index('<world name="disaster">')
    end = header_src.index('<include>\n      <uri>model://disaster_ground</uri>')
    header = header_src[start:end].replace('<world name="disaster">', '<world name="region">')
    payload = header_src[header_src.index('    <!-- SAR payload package'):header_src.index('  </world>')]
    heroes = '\n'.join(
        f'    <include><name>{h["name"]}</name><uri>https://fuel.gazebosim.org/1.0/{h["uri"].split("/")[0]}/models/{h["uri"].split("/")[1]}</uri>'
        f'<pose>{h["x"]:.2f} {h["y"]:.2f} 0 0 0 {h["yaw"]:.2f}</pose><static>true</static></include>' for h in L['hero'])
    fires = []
    for fire in L['fires']:
        fires.append(f'''    <model name="{fire['name']}"><static>true</static><pose>{fire['x']} {fire['y']} 0 0 0 0.3</pose>
      <link name="link">
        <visual name="ash"><geometry><mesh><uri>model://disaster_ground/meshes/rubble_mound_3.dae</uri><scale>2.0 1.8 0.8</scale></mesh></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.13 0.12 0.12 1</diffuse><specular>0 0 0 1</specular></material>
          <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>305.0</temperature></plugin></visual>
        <visual name="beam_1"><pose>0 0 0.3 0 0.08 0</pose><geometry><box><size>2.6 0.3 0.3</size></box></geometry><material><ambient>0.03 0.025 0.02 1</ambient><diffuse>0.05 0.04 0.035 1</diffuse></material><plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>335.0</temperature></plugin></visual>
        <visual name="ember_1"><pose>0.3 0.1 0.35 0 0 0.4</pose><geometry><box><size>0.5 0.25 0.1</size></box></geometry><material><ambient>1 0.3 0.05 1</ambient><diffuse>1 0.3 0.05 1</diffuse><emissive>1 0.35 0.05 1</emissive></material><plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>365.0</temperature></plugin></visual>
      </link>
    </model>''')
        if fire['smoke']:
            fires.append(f'    <include><name>smoke_{fire["name"]}</name><uri>model://smoke_plume</uri><pose>{fire["x"]} {fire["y"]} 3 0 0 0</pose></include>')
    people = []
    for c in casualties:
        wps, _ = trajectory(c)
        people.append(actor(f'casualty_{c["id"].lower()}', c))
        people.append(heat_body(f'casualty_{c["id"].lower()}', c['pose'], wps[0][1], wps[0][2], wps[0][3]))
    staging = '''    <include><name>ambulance</name><uri>model://ambulance</uri><pose>-12 -9 0 0 0 1.57</pose><static>true</static></include>
    <include><name>fire_truck</name><uri>model://fire_truck</uri><pose>-18 -9 0 0 0 1.57</pose><static>true</static></include>'''
    world = f'''<?xml version="1.0" ?>
<!--
  GENERATED by worldgen/generate_region.py (casualty seed {casualties and casualties[0].get("seed", "?")}) -
  edit the generator, not this file. 200 x 200 m region: city (NE),
  forest (NW), village (SW), river + flood (SE); launch pad at the origin.
  Same frame, lights, sky and GPS origin as disaster.sdf.
-->
<sdf version="1.9">
  {header}
    <include><uri>model://region_scenery</uri></include>

    <!-- Hero models (Fuel, render-checked in Harmonic), all static -->
{heroes}

    <!-- Staging at the command post -->
{staging}

    <!-- Fires (thermal hot spots; one with smoke) -->
{chr(10).join(fires)}

    <!-- Casualties (seeded): actor + matching heat body -->
{chr(10).join(people)}

{payload}
  </world>
</sdf>
'''
    for d in (WORLDS_SRC, WORLDS_INSTALL):
        if os.path.isdir(d):
            with open(os.path.join(d, 'region.sdf'), 'w') as f:
                f.write(world)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=1, help='casualty placement seed')
    ap.add_argument('--casualties', type=int, default=6)
    ap.add_argument('--scenery', action='store_true', help='rebuild the scenery model, basemap and mission intel')
    args = ap.parse_args()

    L = build_layout(random.Random(SCENERY_SEED))
    if args.scenery or not os.path.isfile(os.path.join(SCENERY, 'model.sdf')):
        stats = build_scenery(L, np.random.default_rng(SCENERY_SEED))
        build_basemap(L)
        build_prior(L)
        write_mission(L)
        print('scenery:', stats)
    casualties = place_casualties(L, args.seed, args.casualties)
    for c in casualties:
        c['seed'] = args.seed
    write_world(L, casualties)
    movers = []
    for c in casualties:
        if c['pose'] in MOVES:
            wps, period = trajectory(c)
            movers.append({'model': f'casualty_{c["id"].lower()}_heat', 'z': HEAT_Z[c['pose']], 'period': period,
                           'waypoints': [[round(t, 2), round(x, 2), round(y, 2), round(heat_yaw(c['pose'], yaw), 3)] for t, x, y, yaw in wps]})
    with open(ACTORS, 'w') as f:
        json.dump({'world': 'region', 'movers': movers}, f, indent=2)
    with open(TRUTH, 'w') as f:
        json.dump({'seed': args.seed, 'casualties': casualties}, f, indent=2)
    print(f'world: seed {args.seed}, {len(casualties)} casualties: ' +
          ', '.join(f"{c['id']}@({c['x']:.0f},{c['y']:.0f}) {c['where']} {c['pose']}" for c in casualties))


if __name__ == '__main__':
    main()
