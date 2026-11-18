"""evaluate.py - grade a perception file against the simulator's tracks.

No MATLAB, Simulink or Unreal needed: this script builds Pandar40P-like scans of
the track library itself (same CSV files, same sensor settings as sim_params.m),
runs your detect() on every scan and compares the result with the true cone
positions.

    python evaluate.py                          # grade perception.py on all tracks
    python evaluate.py --tracks autocross_01 skidpad
    python evaluate.py --file my_algorithm.py   # grade another file
    python evaluate.py -v                       # one line per scan (for debugging)

It prints a score table and a PASS / FAIL verdict (exit code 0 = pass), so it can
also run in CI. The pass marks are in LIMITS below - agree them with the team.

How a scan is scored
  * A cone only counts if the LiDAR could see it: within SCORE_RANGE of the
    sensor and hit by at least MIN_HITS points. Cones behind other cones or too
    far away are not held against the detector.
  * Detections are matched to those cones one-to-one (Hungarian assignment) with
    a MATCH_GATE distance gate.
        matched detection  = TP      unmatched detection = FP
        unmatched cone     = FN
    Detections further than SCORE_RANGE are ignored (neither TP nor FP).
  * precision = TP/(TP+FP)   recall = TP/(TP+FN)   position error = distance
    between a matched detection and its true cone.
  * FPs that sit on a distractor (spare cone stack, timekeeping equipment) are
    counted separately, so you can see what is fooling the detector.
"""

import argparse
import importlib.util
import inspect
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TRACK_DIR = HERE.parent / "tracks"

# ----- pass marks (edit freely) ---------------------------------------------
LIMITS = {
    "recall":     0.90,    # >= this fraction of visible cones found
    "precision":  0.90,    # >= this fraction of detections are real cones
    "pos_err":    0.15,    # <= mean position error of matched cones [m]
    "runtime_p95": 0.100,  # <= seconds per scan (the LiDAR delivers 10 scans/s)
}

# ----- scoring rules (keep equal to PerceptionScore.m) ------------------------
SCORE_RANGE = 30.0         # only cones / detections closer than this count [m]
MIN_HITS = 3               # LiDAR points a cone needs to count as visible
HIT_RADIUS = 0.25          # point-to-cone distance that counts as a hit [m]
GROUND_TOL = 0.04          # points above ground + this are "objects" [m]
MATCH_GATE = 0.5           # detection and cone further apart than this never match [m]
BANDS = [(0, 10), (10, 20), (20, 30)]

# ----- sensor model (mirrors sim_params.m) -----------------------------------
ELEV = np.array([15, 11, 8, 5, 3, 2, 5/3, 4/3, 1, 2/3, 1/3, 0, -1/3, -2/3, -1, -4/3,
                 -5/3, -2, -7/3, -8/3, -3, -10/3, -11/3, -4, -13/3, -14/3, -5, -16/3,
                 -17/3, -6, -7, -8, -9, -10, -11, -12, -13, -14, -19, -25])   # deg, ring 0..39
AZ = np.arange(-180.0, 180.0, 0.2)       # deg, 1800 columns
SENSOR_Z = 0.40                          # sensor height above ground [m] (0.30 + mount z)
RANGE, MIN_RANGE, NOISE_STD = 80.0, 0.3, 0.02
GEOM = {                                 # [rBottom rTop height] [m]
    "blue": (0.105, 0.020, 0.325), "yellow": (0.105, 0.020, 0.325),
    "orange_small": (0.105, 0.020, 0.325), "orange_big": (0.130, 0.025, 0.505),
    "distractor_stack": (0.105, 0.030, 0.600), "timekeeping": (0.150, 0.150, 1.200),
}
CLASS_OF = {"blue": 0, "yellow": 1, "orange_small": 2, "orange_big": 2}   # as in perception.py


# ----- track + scan generation ------------------------------------------------
def load_track(name):
    import csv
    objs, types = [], []
    with open(TRACK_DIR / f"{name}.csv", newline="") as f:
        for row in csv.DictReader(f):
            objs.append((float(row["x"]), float(row["y"])))
            types.append(row["type"])
    centre, closed = None, False
    try:
        from scipy.io import loadmat
        meta = loadmat(TRACK_DIR / f"{name}_meta.mat", squeeze_me=True)
        centre = np.atleast_2d(meta["centre"]).astype(float)
        closed = str(meta["event"]) in ("autocross", "trackdrive")
    except Exception as err:                                   # noqa: BLE001
        sys.exit(f"cannot read {name}_meta.mat ({err}); scipy is needed for the centreline")
    return np.array(objs), np.array(types), centre, closed


def poses_along(centre, closed, spacing, max_scans):
    """Sensor poses (x, y, yaw) along the centreline, `spacing` metres apart."""
    ds = float(np.median(np.linalg.norm(np.diff(centre, axis=0), axis=1)))
    step = max(1, int(round(spacing / ds)))
    n = len(centre)
    idx = np.arange(0, n if closed else n - 5, step)
    if len(idx) > max_scans:
        idx = idx[np.linspace(0, len(idx) - 1, max_scans).astype(int)]
    out = []
    for i in idx:
        j = (i + 4) % n if closed else min(i + 4, n - 1)
        d = centre[j] - centre[i]
        out.append((centre[i, 0], centre[i, 1], float(np.arctan2(d[1], d[0]))))
    return out


def scan(pose, objs, geom, rng):
    """One Pandar40P-like scan. Returns points (N,3), intensity (N,), ring (N,)."""
    x0, y0, yaw = pose
    el, az = np.deg2rad(ELEV), np.deg2rad(AZ)
    nE, nA = len(el), len(az)
    ce, se = np.cos(el)[:, None], np.sin(el)[:, None]
    dx = ce * np.cos(az)[None, :]
    dy = ce * np.sin(az)[None, :]
    dz = np.broadcast_to(se, (nE, nA))
    hit = np.full((nE, nA), np.inf)

    down = se[:, 0] < 0                                    # ground plane z = -SENSOR_Z
    hit[down, :] = (SENSOR_Z / -se[down, 0])[:, None]

    c, s = np.cos(yaw), np.sin(yaw)
    rel = objs - np.array([x0, y0])
    ox, oy = c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]   # sensor frame
    azres = np.deg2rad(AZ[1] - AZ[0])
    for k in np.flatnonzero(np.hypot(ox, oy) < RANGE):
        rb, rt, h = geom[k]
        dist = np.hypot(ox[k], oy[k])
        if dist < 0.05:
            continue
        w = np.arcsin(min(1.0, (rb + 0.05) / dist)) + azres
        cols = np.flatnonzero(np.abs((az - np.arctan2(oy[k], ox[k]) + np.pi) % (2 * np.pi) - np.pi) <= w)
        if cols.size == 0:
            continue
        ddx, ddy, ddz = dx[:, cols], dy[:, cols], dz[:, cols]
        kk = (rb - rt) / h
        px, py, pz = -ox[k], -oy[k], SENSOR_Z              # ray origin relative to the cone base
        a0, b0 = rb - kk * pz, -kk * ddz                   # cone radius = a0 + b0 * t
        qa = ddx ** 2 + ddy ** 2 - b0 ** 2
        qb = 2 * (px * ddx + py * ddy - a0 * b0)
        qc = px ** 2 + py ** 2 - a0 ** 2
        disc = qb ** 2 - 4 * qa * qc
        good = (disc >= 0) & (np.abs(qa) > 1e-12)
        sq = np.sqrt(np.where(good, disc, 0.0))
        qa = np.where(good, qa, 1.0)
        ta, tb = (-qb - sq) / (2 * qa), (-qb + sq) / (2 * qa)
        t1, t2 = np.minimum(ta, tb), np.maximum(ta, tb)
        ok1 = (t1 > 0) & (pz + t1 * ddz >= 0) & (pz + t1 * ddz <= h)
        ok2 = (t2 > 0) & (pz + t2 * ddz >= 0) & (pz + t2 * ddz <= h)
        t = np.where(ok1, t1, np.where(ok2, t2, np.inf))
        t = np.where(good & (t <= RANGE), t, np.inf)
        hit[:, cols] = np.minimum(hit[:, cols], t)

    valid = np.isfinite(hit) & (hit >= MIN_RANGE) & (hit <= RANGE)
    r = hit[valid] + NOISE_STD * rng.standard_normal(valid.sum())
    pts = np.column_stack([r * dx[valid], r * dy[valid], r * dz[valid]]).astype(np.float32)
    ring = np.broadcast_to(np.arange(nE)[:, None], (nE, nA))[valid].astype(np.int32)
    return pts, np.full(len(pts), 0.5, np.float32), ring


# ----- scoring ----------------------------------------------------------------
def ground_z(z):
    """Most common height = the ground (same method as PointCloudViewer.m)."""
    edges = np.arange(np.floor(z.min() / 0.02) * 0.02, z.max() + 0.04, 0.02)
    if len(edges) < 2:
        return float(np.median(z))
    n, e = np.histogram(z, edges)
    k = int(np.argmax(n))
    return 0.5 * (e[k] + e[k + 1])


def visible_cones(pts, cone_xy):
    """Boolean per cone: close enough and hit by at least MIN_HITS points."""
    obj = pts[pts[:, 2] > ground_z(pts[:, 2]) + GROUND_TOL]
    vis = np.zeros(len(cone_xy), bool)
    for k, p in enumerate(cone_xy):
        if np.hypot(*p) <= SCORE_RANGE and len(obj):
            vis[k] = np.count_nonzero(np.hypot(obj[:, 0] - p[0], obj[:, 1] - p[1]) < HIT_RADIUS) >= MIN_HITS
    return vis


def assign(det_xy, truth_xy):
    """One-to-one matches within MATCH_GATE -> list of (det index, truth index)."""
    if len(det_xy) == 0 or len(truth_xy) == 0:
        return []
    d = np.linalg.norm(det_xy[:, None, :] - truth_xy[None, :, :], axis=2)
    try:
        from scipy.optimize import linear_sum_assignment
        r, c = linear_sum_assignment(np.where(d <= MATCH_GATE, d, 1e6))
        return [(i, j) for i, j in zip(r, c) if d[i, j] <= MATCH_GATE]
    except ImportError:                                        # greedy fallback
        pairs, used_r, used_c = [], set(), set()
        for flat in np.argsort(d, axis=None):
            i, j = divmod(int(flat), d.shape[1])
            if d[i, j] > MATCH_GATE:
                break
            if i not in used_r and j not in used_c:
                pairs.append((i, j)); used_r.add(i); used_c.add(j)
        return pairs


def load_detector(path):
    spec = importlib.util.spec_from_file_location(Path(path).stem, path)
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(Path(path).resolve().parent))
    spec.loader.exec_module(mod)
    if not hasattr(mod, "detect"):
        sys.exit(f"{path} has no detect() function")
    return mod.detect, min(max(len(inspect.signature(mod.detect).parameters), 1), 3)


def run_detector(fn, nargs, pts, inten, ring):
    t0 = time.perf_counter()
    res = fn(*(pts, inten, ring)[:nargs])
    dt = time.perf_counter() - t0
    det = np.zeros((0, 4)) if res is None else np.asarray(res, dtype=float)
    if det.size == 0:
        det = np.zeros((0, 4))
    det = det.reshape(1, -1) if det.ndim == 1 else det
    if det.shape[1] < 3:
        sys.exit("detect() must return rows of [x y z] or [x y z class]")
    if det.shape[1] == 3:                                      # no class: side decides (as in the sim)
        det = np.column_stack([det, (det[:, 1] < 0).astype(float)])
    return det[:, :4], dt


class Tally:
    def __init__(self):
        self.scans = self.visible = self.tp = self.fp = self.fn = self.distractor_fp = 0
        self.err, self.colour_ok, self.times = [], 0, []
        self.band_tp = np.zeros(len(BANDS), int)
        self.band_n = np.zeros(len(BANDS), int)

    def add(self, other):
        for k in ("scans", "visible", "tp", "fp", "fn", "distractor_fp", "colour_ok"):
            setattr(self, k, getattr(self, k) + getattr(other, k))
        self.err += other.err
        self.times += other.times
        self.band_tp += other.band_tp
        self.band_n += other.band_n

    precision = property(lambda s: s.tp / (s.tp + s.fp) if s.tp + s.fp else float("nan"))
    recall = property(lambda s: s.tp / (s.tp + s.fn) if s.tp + s.fn else float("nan"))
    pos_err = property(lambda s: float(np.mean(s.err)) if s.err else float("nan"))
    colour = property(lambda s: s.colour_ok / s.tp if s.tp else float("nan"))
    p95 = property(lambda s: float(np.percentile(s.times, 95)) if s.times else float("nan"))


def score_track(name, fn, nargs, spacing, max_scans, seed, verbose):
    objs, types, centre, closed = load_track(name)
    geom = np.array([GEOM[t] for t in types])
    is_cone = np.array([t in CLASS_OF for t in types])
    is_dist = ~is_cone
    cls = np.array([CLASS_OF.get(t, -1) for t in types])
    rng = np.random.default_rng(seed)
    tally = Tally()
    for pose in poses_along(centre, closed, spacing, max_scans):
        pts, inten, ring = scan(pose, objs, geom, rng)
        c, s = np.cos(pose[2]), np.sin(pose[2])
        rel = objs - np.array(pose[:2])
        xy = np.column_stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]])  # sensor frame
        vis = is_cone & (np.hypot(xy[:, 0], xy[:, 1]) <= SCORE_RANGE)
        vis[is_cone] = visible_cones(pts, xy[is_cone]) & vis[is_cone]
        det, dt = run_detector(fn, nargs, pts, inten, ring)
        det = det[np.isfinite(det[:, :3]).all(axis=1)]
        det = det[np.hypot(det[:, 0], det[:, 1]) <= SCORE_RANGE]

        tru_idx = np.flatnonzero(vis)
        pairs = assign(det[:, :2], xy[tru_idx])
        matched_det = {i for i, _ in pairs}
        matched_tru = {j for _, j in pairs}
        tally.scans += 1
        tally.visible += len(tru_idx)
        tally.tp += len(pairs)
        tally.fn += len(tru_idx) - len(pairs)
        tally.fp += len(det) - len(pairs)
        tally.times.append(dt)
        for i in set(range(len(det))) - matched_det:          # false positives on a distractor?
            if is_dist.any() and np.min(np.hypot(*(xy[is_dist] - det[i, :2]).T)) < 0.4:
                tally.distractor_fp += 1
        for i, j in pairs:
            tally.err.append(float(np.hypot(*(det[i, :2] - xy[tru_idx[j], :2]))))
            tally.colour_ok += int(round(det[i, 3]) == cls[tru_idx[j]])
        for b, (lo, hi) in enumerate(BANDS):
            in_band = [j for j, k in enumerate(tru_idx) if lo <= np.hypot(*xy[k]) < hi]
            tally.band_n[b] += len(in_band)
            tally.band_tp[b] += sum(j in matched_tru for j in in_band)
        if verbose:
            print(f"  {name} scan {tally.scans:3d}: {len(pts):6d} pts  visible {len(tru_idx):2d}  "
                  f"detected {len(det):2d}  TP {len(pairs):2d}  FP {len(det) - len(pairs):2d}  "
                  f"FN {len(tru_idx) - len(pairs):2d}  {dt * 1000:6.1f} ms")
    return tally


def pct(x):
    return "  n/a" if x != x else f"{100 * x:4.0f}%"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=str(HERE / "perception.py"), help="python file with detect()")
    ap.add_argument("--tracks", nargs="*", help="track names (default: every track in tracks/)")
    ap.add_argument("--spacing", type=float, default=5.0, help="distance between scans along the track [m]")
    ap.add_argument("--max-scans", type=int, default=40, help="most scans per track")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    names = a.tracks or sorted(p.stem for p in TRACK_DIR.glob("*.csv"))
    fn, nargs = load_detector(a.file)
    print(f"grading {a.file}  ({nargs} argument{'s' if nargs > 1 else ''})\n")

    total, rows = Tally(), []
    for name in names:
        t = score_track(name, fn, nargs, a.spacing, a.max_scans, a.seed, a.verbose)
        total.add(t)
        rows.append((name, t))

    print(f"{'track':<16}{'scans':>6}{'cones':>7}{'TP':>6}{'FP':>5}{'FN':>5}"
          f"{'prec':>7}{'recall':>8}{'err m':>8}{'colour':>8}{'p95 ms':>8}")
    for name, t in rows + [("ALL", total)]:
        if name == "ALL":
            print("-" * 84)
        print(f"{name:<16}{t.scans:>6}{t.visible:>7}{t.tp:>6}{t.fp:>5}{t.fn:>5}"
              f"{pct(t.precision):>7}{pct(t.recall):>8}{t.pos_err:>8.3f}{pct(t.colour):>8}{t.p95 * 1000:>8.1f}")

    print("\nrecall by distance:  " + "   ".join(
        f"{lo}-{hi} m {pct(total.band_tp[b] / total.band_n[b] if total.band_n[b] else float('nan'))}"
        for b, (lo, hi) in enumerate(BANDS)))
    print(f"false positives on distractors (spare cones / timekeeping): {total.distractor_fp} of {total.fp}")
    if total.err:
        e = np.array(total.err)
        print(f"position error:  mean {e.mean():.3f} m   median {np.median(e):.3f} m   max {e.max():.3f} m")

    checks = [
        ("recall",      total.recall,    ">=", LIMITS["recall"],      pct),
        ("precision",   total.precision, ">=", LIMITS["precision"],   pct),
        ("position error [m]", total.pos_err, "<=", LIMITS["pos_err"], lambda v: f"{v:.3f}"),
        ("runtime p95 [s]",    total.p95,     "<=", LIMITS["runtime_p95"], lambda v: f"{v:.3f}"),
    ]
    print("\npass marks")
    ok_all = True
    for label, val, op, lim, fmt in checks:
        ok = (val >= lim) if op == ">=" else (val <= lim)
        ok_all &= bool(ok)
        print(f"  {'PASS' if ok else 'FAIL'}  {label:<20} {fmt(val):>7}  (needs {op} {fmt(lim)})")
    print(f"\nRESULT: {'PASS' if ok_all else 'FAIL'}")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
