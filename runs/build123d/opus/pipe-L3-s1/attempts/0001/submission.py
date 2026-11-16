"""Pipe-to-square-mount bracket, pipe-L3-s1 (build123d).

Design intent
-------------
Printed standing on its flat foot with the pipe axis vertical, so that
nearly every face is vertical or faces up:

* saddle  - a prism extruded along the pipe axis: a concave seat on the
  pipe (R34) and two flat facets, each square to one row of M5 bolts, so
  every cap-screw head sits on a proper spot face.
* web     - a vertical web from the saddle to the square mount.  Its foot
  is on the bed with the saddle's foot; it runs between the two columns
  of M4 bolts so all keys and drivers stay clear.
* plate   - a 4 mm flange on B's face, seat kept 1.2 mm inside B's
  outline; its lower edge is bevelled so it is not a flat overhang.

All modelling is done in a local frame  X = c1 (B's horizontal in-plane
axis), Y = e (horizontal, from B towards the pipe), Z = d (pipe axis,
"up" when printing), origin at the centre of B's mounting face, and the
result is moved into world coordinates at the end.
"""
import json
import math
import pathlib

import numpy as np
from build123d import (
    Axis, BuildLine, BuildSketch, Box, Cylinder, Line, Location, Mode, Part,
    Plane, Polyline, ThreePointArc, Vector, export_step, extrude, fillet,
    make_face, Pos, Rot,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
RES = POSE["resolved"]
BOLTS = {b["id"]: b for b in RES["bolts"]}

# ---------------------------------------------------------------- frame
RA = np.array(POSE["placements"]["A"]["rotation"])
d = RA[:, 0]                                  # pipe axis (print "up")
C_A = np.array(POSE["placements"]["A"]["origin"])  # on axis, mid of bolts
FB = RES["mounting_faces"][0]
Bc = np.array(FB["origin"])
n = np.array(FB["normal"])                    # B face normal, into our part
e = n - (n @ d) * d
e /= np.linalg.norm(e)
c1 = np.cross(e, d)                           # so (c1, e, d) is right-handed
c1 /= np.linalg.norm(c1)


def loc(p):
    """world point -> local (c1, e, d) coordinates"""
    q = np.asarray(p, float) - Bc
    return np.array([q @ c1, q @ e, q @ d])


def ldir(v):
    v = np.asarray(v, float)
    return np.array([v @ c1, v @ e, v @ d])


N = ldir(n)                       # (0, 0.768, 0.640)
C0 = np.cross(N, [1.0, 0, 0])     # B's in-plane axis that rises with d
if C0[2] < 0:
    C0 = -C0
PC = loc(C_A)                     # pipe axis point (local)
R_PIPE = 34.0

# ------------------------------------------------------------ parameters
G_A = 5.0          # saddle thickness under M5 heads (grip)
G_B = 4.0          # flange thickness under M4 heads (grip)
HOLE_A = 5.5
HOLE_B = 4.5
PAD_A = 6.5        # facet length beyond each bolt axis
SEAT_B = 35.6      # flange seat size (B face is 38, keep >= 1 mm inside)
WEB_T = 13.0       # web thickness
H_MARGIN = 6.4     # saddle length beyond the outer bolts' axes
OFF_B = 1.5        # web set back from B's face plane below the flange

A_ids = ["A1", "A2", "A3", "A4"]
h_bolts = [loc(BOLTS[i]["position"])[2] for i in A_ids]
H0 = min(h_bolts) - H_MARGIN      # bed level
H1 = max(h_bolts) + H_MARGIN      # top of saddle


def nc(nv, c0v):
    """(n, c0) in B's frame -> local (e, h) in the web's elevation"""
    p = nv * N + c0v * C0
    return p[1], p[2]


def saddle():
    pc = PC[:2]
    rows = {}
    for i in ("A1", "A3"):
        u = ldir(BOLTS[i]["axis"])[:2]
        u /= np.linalg.norm(u)
        rows[i] = u
    u1, u3 = rows["A1"], rows["A3"]
    D = R_PIPE + G_A
    # peak: intersection of the two facet lines  (p-pc).u = D
    M = np.array([u1, u3])
    peak = pc + np.linalg.solve(M, [D, D])

    def facet_end(u, other):
        t = np.array([-u[1], u[0]])
        # tangent pointing away from the other row
        if t @ other > 0:
            t = -t
        return pc + D * u + PAD_A * t

    E1 = facet_end(u1, u3)
    E3 = facet_end(u3, u1)

    def to_arc(p):
        v = p - pc
        return pc + R_PIPE * v / np.linalg.norm(v)

    I1, I3 = to_arc(E1), to_arc(E3)
    mid = pc + R_PIPE * (u1 + u3) / np.linalg.norm(u1 + u3)
    with BuildSketch(Plane.XY.offset(H0)) as sk:
        with BuildLine():
            Polyline(tuple(I3), tuple(E3), tuple(peak), tuple(E1), tuple(I1))
            ThreePointArc(tuple(I1), tuple(mid), tuple(I3))
        make_face()
    return extrude(sk.sketch, amount=H1 - H0), peak


def web(peak):
    # top view: band from the flange centre straight out, then to the saddle
    pc = PC[:2]
    toward = (peak - pc) / np.linalg.norm(peak - pc)
    deep = pc + (R_PIPE - 4) * toward          # end buried in the pipe
    path = [np.array([0.0, -12.0]), np.array([0.0, 38.0]), deep]
    hw = WEB_T / 2
    left, right = [], []
    for k, p in enumerate(path):
        if k == 0:
            t = path[1] - path[0]
        elif k == len(path) - 1:
            t = path[-1] - path[-2]
        else:
            t1 = (path[k] - path[k - 1]); t1 /= np.linalg.norm(t1)
            t2 = (path[k + 1] - path[k]); t2 /= np.linalg.norm(t2)
            t = t1 + t2
        t = t / np.linalg.norm(t)
        nn = np.array([-t[1], t[0]])
        s = 1.0
        if 0 < k < len(path) - 1:     # mitre
            t1 = (path[k] - path[k - 1]); t1 /= np.linalg.norm(t1)
            s = 1.0 / (nn @ np.array([-t1[1], t1[0]]))
        left.append(p + hw * s * nn)
        right.append(p - hw * s * nn)
    pts = left + right[::-1]
    with BuildSketch(Plane.XY.offset(H0)) as sk:
        with BuildLine():
            Polyline(*[tuple(p) for p in pts], close=True)
        make_face()
    plan = extrude(sk.sketch, amount=H1 - H0)

    # elevation (e, h) in the plane X = 0
    top_f = nc(OFF_B, SEAT_B / 2 - 1.8)
    # point where the set-back line meets the bed
    c0_bed = (H0 - OFF_B * N[2]) / C0[2]
    bed_f = nc(OFF_B, c0_bed)
    far = PC[1] + 10
    peak_e = peak[1]
    prof = [top_f, bed_f, (far, H0), (far, H1), (peak_e, H1)]
    with BuildSketch(Plane.YZ.offset(-60)) as sk2:
        with BuildLine():
            Polyline(*prof, close=True)
        make_face()
    elev = extrude(sk2.sketch, amount=120)
    return plan & elev


def plate():
    s = SEAT_B / 2
    # build in a (c1, c0, n) frame then place
    p = Box(SEAT_B, SEAT_B, G_B)
    p = Pos(0, 0, G_B / 2) * p
    p = fillet(p.edges().filter_by(Axis.Z), radius=3.0)
    # bevel the lower edge (c0 = -s) so its face is not a flat overhang
    beta = math.radians(14)
    cutter = Box(60, 20, 30)
    cutter = Rot(-math.degrees(beta), 0, 0) * Pos(0, -10, 0) * cutter
    cutter = Pos(0, -s, 0) * cutter
    p = p - cutter
    pl = Plane(origin=(0, 0, 0), x_dir=(-1, 0, 0), z_dir=tuple(N))
    # plane y-dir must be C0
    assert np.allclose(np.array(tuple(pl.y_dir)), C0, atol=1e-6)
    return pl.location * p


def build() -> Part:
    sad, peak = saddle()
    body = sad + web(peak) + plate()
    # pipe
    body -= Pos(*PC[:2], 0) * Cylinder(R_PIPE, 600)
    # holes
    for bid, b in BOLTS.items():
        p = loc(b["position"])
        a = ldir(b["axis"])
        dia = HOLE_A if b["size"] == "M5" else HOLE_B
        pl = Plane(origin=tuple(p - 2 * a), z_dir=tuple(a))
        body -= pl.location * Cylinder(dia / 2, 40, align=(
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.MIN))
    # to world
    world = Plane(origin=tuple(Bc), x_dir=tuple(c1), z_dir=tuple(d))
    return world.location * body


if __name__ == "__main__":
    part = build()
    print("solids", len(part.solids()), "volume", round(part.volume, 1))
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
