"""perception.py - cone detection for the SUFST driverless simulator.

This file is loaded by the "Perception (Python)" block in fs_driverless_top.
It is a BASELINE (ground removal + clustering, numpy only). The perception
team replaces it with their own algorithm - keep the function signature and
the return format and nothing else in the model has to change.

The block calls, once per LiDAR scan (10 Hz):

    detect(points, intensity, ring, timestamp, ego)

    points     (N, 3) float32  x forward, y left, z up [m], SENSOR frame.
                               Only valid returns (no NaN).
    intensity  (N,)   float32  reflectivity of each point (simulated, 0..1)
    ring       (N,)   int32    channel number of each point, 0 .. 39, in the
                               Pandar40P channel order (lidarCfg.elev in
                               sim_params.m): 0 = top beam (+15 deg),
                               39 = bottom beam (-25 deg)

    timestamp  (N,)   float32  when each point was measured within the 0.1 s
                               sweep [s]. Like the real Hesai driver, the cloud is
                               NOT corrected for the car's motion: a car at 15 m/s
                               moves 1.5 m during the sweep, so points measured late
                               are shifted. Use timestamp (and the car's speed) to
                               de-skew if you want to.
    ego        (3,)   float64  the car's velocity during the sweep, body frame:
                               [vx forward m/s, vy left m/s, yaw rate rad/s] -
                               what wheel-speed / IMU odometry gives.

    Declaring fewer arguments is fine (detect(points), detect(points, intensity),
    ... up to all five).

    returns    (M, 4) array / list of rows [x, y, z, cls] in the SENSOR frame
               (M may be 0). cls:  0 = left cone  (blue)
                                   1 = right cone (yellow)
                                   2 = orange cone
                                   3 = unknown
               A (M, 3) result is also accepted: cls is then set from the
               side the cone is on (y > 0 -> left, y < 0 -> right).

The simulator turns this into: number of cones, number on the left, number on
the right, number of other, and the cone list itself.

Run the file directly (python perception.py) for a quick self-test.
"""

import numpy as np

# ----- parameters of the baseline detector (tune freely) --------------------
MAX_RANGE = 40.0       # ignore everything further than this [m]
GROUND_BIN = 0.02      # height histogram bin used to find the ground [m]
Z_MIN = 0.03           # a point must be this far above the ground ...
Z_MAX = 0.70           # ... and below this to be a cone candidate [m]
CELL = 0.30            # clustering grid cell [m]
MIN_POINTS = 3         # smallest cluster accepted as a cone
MAX_EXTENT = 0.60      # biggest cluster footprint accepted as a cone [m]
MIN_HEIGHT = 0.08      # a cone cluster must stick up at least this far [m]
MAX_HEIGHT = 0.60      # ... and no more than this [m]

# Motion de-skew: a spinning LiDAR measures each point at a different time while the
# car moves; at speed that smears the cloud. With the per-point timestamp and the
# car's velocity we can move every point back to the sensor frame at the sweep start.
DESKEW = True
SENSOR_XY = (1.2, 0.0)   # sensor position in the car frame, from the pose reference [m]

LEFT, RIGHT, ORANGE, UNKNOWN = 0, 1, 2, 3
_NEIGHBOURS = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1)]


def _ground_height(z):
    """Ground = the most common height (the ground is most of the scan)."""
    lo, hi = np.percentile(z, [1, 60])
    bins = np.arange(lo - GROUND_BIN, hi + 2 * GROUND_BIN, GROUND_BIN)
    hist, edges = np.histogram(z, bins)
    k = int(np.argmax(hist))
    return 0.5 * (edges[k] + edges[k + 1])


def deskew(pts, t, ego, sensor_xy=SENSOR_XY):
    """Re-express every point in the sensor frame at the start of the sweep."""
    vx, vy, om = (float(v) for v in ego)
    t = np.asarray(t, dtype=np.float64)
    wt = om * t
    c, s = np.cos(wt), np.sin(wt)
    if abs(om) > 1e-6:
        dx = (vx * s - vy * (1 - c)) / om
        dy = (vx * (1 - c) + vy * s) / om
    else:
        dx, dy = vx * t, vy * t
    # the sensor also swings round the car's reference point when the car yaws
    dx = dx + (c - 1) * sensor_xy[0] - s * sensor_xy[1]
    dy = dy + s * sensor_xy[0] + (c - 1) * sensor_xy[1]
    out = pts.copy()
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + dx
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + dy
    return out


def detect(points, intensity=None, ring=None, timestamp=None, ego=None):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if DESKEW and timestamp is not None and ego is not None:
        pts = deskew(pts, timestamp, ego)
    if pts.shape[0] < MIN_POINTS:
        return np.zeros((0, 4))

    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < MAX_RANGE]
    if pts.shape[0] < MIN_POINTS:
        return np.zeros((0, 4))

    # 1. remove the ground
    z0 = _ground_height(pts[:, 2])
    h = pts[:, 2] - z0
    cand = pts[(h > Z_MIN) & (h < Z_MAX)]
    h = h[(h > Z_MIN) & (h < Z_MAX)]
    if cand.shape[0] < MIN_POINTS:
        return np.zeros((0, 4))

    # 2. cluster the remaining points: connected cells on a 2-D grid
    cells = np.floor(cand[:, :2] / CELL).astype(np.int64)
    occupied = {}
    for idx, (i, j) in enumerate(map(tuple, cells)):
        occupied.setdefault((i, j), []).append(idx)

    seen, clusters = set(), []
    for start in occupied:
        if start in seen:
            continue
        stack, members = [start], []
        seen.add(start)
        while stack:
            c = stack.pop()
            members.extend(occupied[c])
            for di, dj in _NEIGHBOURS:
                n = (c[0] + di, c[1] + dj)
                if n in occupied and n not in seen:
                    seen.add(n)
                    stack.append(n)
        clusters.append(members)

    # 3. keep the clusters that look like a cone
    out = []
    for members in clusters:
        if len(members) < MIN_POINTS:
            continue
        p, hh = cand[members], h[members]
        extent = np.ptp(p[:, :2], axis=0).max()
        if extent > MAX_EXTENT or not (MIN_HEIGHT <= hh.max() <= MAX_HEIGHT):
            continue
        cx, cy = p[:, 0].mean(), p[:, 1].mean()
        # LiDAR only: no colour, so left/right is judged from the side
        out.append([cx, cy, z0, LEFT if cy > 0 else RIGHT])

    return np.array(out, dtype=np.float64).reshape(-1, 4)


if __name__ == "__main__":                      # self-test with fake cones
    rng = np.random.default_rng(0)
    ground = np.column_stack([rng.uniform(-30, 30, 20000),
                              rng.uniform(-30, 30, 20000),
                              np.full(20000, -0.30)])
    cones = []
    for cx, cy in [(5, 2), (5, -2), (10, 2.1), (10, -2.1), (20, 2)]:
        a = rng.uniform(0, 2 * np.pi, 12)
        zc = rng.uniform(-0.30, 0.0, 12)
        cones.append(np.column_stack([cx + 0.08 * np.cos(a),
                                      cy + 0.08 * np.sin(a), zc]))
    cloud = np.vstack([ground] + cones).astype(np.float32)
    res = detect(cloud)
    print("expected 5 cones (3 left, 2 right) ->", len(res), "found")
    print(np.round(res, 2))
