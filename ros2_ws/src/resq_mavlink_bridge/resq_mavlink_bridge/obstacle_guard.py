"""Obstacle-aware altitude control from a forward lidar and a downward rangefinder.

No ROS here (the bridge feeds it numpy arrays), so it can be unit-tested.

Every forward-lidar scan is levelled with the drone's roll and pitch, then
cropped to a corridor along the flight path: CORRIDOR_HALF_WIDTH_M either
side, out to a look-ahead distance that grows with speed. Anything in that
corridor standing more than GROUND_TOL_M above the ground below the drone
is an obstacle (a building, tree, pole, or rising terrain). The drone must
fly CLEARANCE_M above the tallest one.

Two refinements:
- A lidar only sees so far up (its top beam is +20 deg). If an obstacle
  fills that top beam, its real top may be higher still, so it is treated
  as at least CLIMB_STEP_M above what is visible.
- If an obstacle at the drone's own level is within the stopping distance,
  the path is *blocked*: the drone holds position and climbs until the
  corridor clears, rather than flying on while climbing.

The downward rangefinder gives the height above whatever is directly below
(ground or a roof), so the drone never descends onto a roof it is crossing.
The commanded altitude is held for HOLD_S after an obstacle leaves the
corridor, so the drone doesn't dip between closely spaced buildings
(8 s: at 3 s it bounced up and down along the region world's city streets).
"""
import math

import numpy as np

CORRIDOR_HALF_WIDTH_M = 2.5
MIN_LOOKAHEAD_M = 10.0
MAX_LOOKAHEAD_M = 35.0
LOOKAHEAD_S = 6.0
GROUND_TOL_M = 1.0
CLEARANCE_M = 4.0
BELOW_CLEARANCE_M = 3.0
CLIMB_STEP_M = 3.0
BLOCK_MARGIN_M = 1.0
CLIMB_RATE_MS = 2.0   # conservative vs ArduCopter's default 2.5 m/s climb
TOP_BEAM_MARGIN_RAD = 0.05
HOLD_S = 8.0   # was 3 s: in the region city the drone bounced up and down between blocks
MAX_ALT_M = 45.0
PROFILE_BIN_M = 2.0


def level_points(points, roll, pitch):
    """Sensor-frame points (x forward, y left, z up; N x 3) -> level frame
    (forward, right, up) relative to the drone, removing roll and pitch
    (ArduPilot convention: roll right-wing-down +, pitch nose-up +)."""
    x, y, z = points[:, 0], -points[:, 1], -points[:, 2]  # FLU -> FRD
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z
    f = cp * x + sp * z1
    d = -sp * x + cp * z1
    return np.column_stack([f, y1, -d])


class ObstacleGuard:

    def __init__(self, top_beam_rad):
        self.top_beam_rad = top_beam_rad
        self.agl = None             # height above what is directly below
        self.required_fwd = 0.0     # altitude (rel. home) the corridor ahead needs
        self.required_until = 0.0   # hold required_fwd until this time
        self.blocked = False
        self.ahead_m = None         # distance to the nearest obstacle in the corridor
        self.top_m = None           # height of the tallest one (rel. home)
        self.top_unknown = False    # its top is above the lidar's view
        self.profile = []           # [[distance, top height]] per bin, for the dashboard
        self.last_scan = 0.0

    def update_scan(self, points, alt, roll, pitch, speed, now):
        """points: N x 3 sensor-frame lidar returns (finite, in range).
        alt: drone altitude above home (m)."""
        self.last_scan = now
        lookahead = min(MAX_LOOKAHEAD_M, max(MIN_LOOKAHEAD_M, speed * LOOKAHEAD_S + 6.0))
        lv = level_points(points, roll, pitch) if len(points) else np.zeros((0, 3))
        f, r, up = lv[:, 0], lv[:, 1], lv[:, 2]
        height = alt + up  # above home
        # Ground (or roof) level under the drone; home level if unknown.
        ground = alt - self.agl if self.agl is not None else 0.0
        in_corridor = (f > 0.5) & (f < lookahead) & (np.abs(r) < CORRIDOR_HALF_WIDTH_M)
        obstacle = in_corridor & (height > ground + GROUND_TOL_M)

        self.profile = []
        if not np.any(obstacle):
            self.blocked = False
            self.ahead_m = None
            self.top_m = None
            self.top_unknown = False
            return
        fo, ho = f[obstacle], height[obstacle]
        for b in range(int(lookahead // PROFILE_BIN_M) + 1):
            m = (fo >= b * PROFILE_BIN_M) & (fo < (b + 1) * PROFILE_BIN_M)
            if np.any(m):
                self.profile.append([round((b + 0.5) * PROFILE_BIN_M, 1), round(float(ho[m].max()), 2)])

        # Rays in the top beam that hit something: the obstacle may go higher.
        elevation = np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1]))
        top_hits = obstacle & (elevation > self.top_beam_rad - TOP_BEAM_MARGIN_RAD)
        self.top_unknown = bool(np.any(top_hits))

        top = float(ho.max())
        required = top + CLEARANCE_M
        if self.top_unknown:
            required = max(required, alt + CLIMB_STEP_M)
        self.top_m = top
        self.ahead_m = float(fo.min())
        # Blocked: an obstacle the drone can't climb over in the time it
        # takes to reach it. Climb needed = its top + clearance - altitude
        # (less BLOCK_MARGIN_M of slack, or noise on the top makes this
        # flicker right at the clear height); climb available = CLIMB_RATE_MS
        # x time to reach it at the current speed. So a 1-2 m climb over a
        # hut ahead happens on the move, while a tall wall close ahead
        # still stops the drone to climb in place.
        need = height + CLEARANCE_M - alt
        unknown = top_hits if self.top_unknown else np.zeros_like(obstacle)
        need = np.where(unknown, np.maximum(need, CLIMB_STEP_M), need)
        time_to_reach = f / max(speed, 1.0)
        cant_make_it = obstacle & (need - BLOCK_MARGIN_M > CLIMB_RATE_MS * time_to_reach)
        # An obstacle whose top is above the lidar's view (a tall wall) can't
        # be timed: within stopping distance it always blocks.
        stop_dist = max(6.0, speed * 3.0)
        tall_close = unknown & (f < stop_dist)
        self.blocked = bool(np.any(cant_make_it) or np.any(tall_close))
        self.required_fwd = min(MAX_ALT_M, required)
        self.required_until = now + HOLD_S

    def update_range(self, distance):
        self.agl = distance if distance is not None and math.isfinite(distance) else None

    def command_altitude(self, search_alt, alt, now):
        """Altitude (rel. home) to fly at: the search altitude, raised for
        whatever is ahead or below."""
        target = search_alt
        if now < self.required_until:
            target = max(target, self.required_fwd)
        if self.agl is not None:
            below_top = alt - self.agl
            if below_top > GROUND_TOL_M:  # over a roof / obstacle, not ground
                target = max(target, below_top + BELOW_CLEARANCE_M, min(alt, below_top + CLEARANCE_M))
        return min(MAX_ALT_M, target)

    def to_dict(self, command_alt, search_alt):
        return {
            'cmd_alt': round(command_alt, 2),
            'search_alt': search_alt,
            'raised': command_alt > search_alt + 0.3,
            'blocked': self.blocked,
            'ahead_m': round(self.ahead_m, 1) if self.ahead_m is not None else None,
            'top_m': round(self.top_m, 1) if self.top_m is not None else None,
            'top_unknown': self.top_unknown,
            'agl': round(self.agl, 2) if self.agl is not None else None,
            'profile': self.profile,
        }
