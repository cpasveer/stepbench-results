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
D_MID = 43.0       # saddle crown flat, distance from the pipe axis
SEAT_B = 35.6      # flange width (B face is 38, keep >= 1 mm inside)
SEAT_LO = -16.6    # flange seat lower edge (c0); B face edge is at -19
APRON = 4.0        # deg the apron below the seat turns away from B's face
WEB_T = 13.0       # web thickness
WEB_TOP = 16.0     # web top height at the saddle (local h)
H_MARGIN = 6.4     # saddle length beyond the outer bolts' axes
OFF_B = 1.5        # web front set back from B's face plane

A_ids = ["A1", "A2", "A3", "A4"]
h_bolts = [loc(BOLTS[i]["position"])[2] for i in A_ids]
H0 = min(h_bolts) - H_MARGIN      # bed level
H1 = max(h_bolts) + H_MARGIN      # top of saddle
NE = np.array([N[1], N[2]])       # n and c0 seen in the (e, h) elevation
CE = np.array([C0[1], C0[2]])


def nc(nv, c0v):
    """(n, c0) in B's frame -> local (e, h) in the web's elevation"""
    return nv * NE + c0v * CE


def poly_face(plane, pts):
    with BuildSketch(plane) as sk:
        with BuildLine():
            Polyline(*[tuple(map(float, p)) for p in pts], close=True)
        make_face()
    return sk.sketch


def saddle():
    pc = PC[:2]
    u1 = ldir(BOLTS["A1"]["axis"])[:2]
    u3 = ldir(BOLTS["A3"]["axis"])[:2]
    u1 /= np.linalg.norm(u1)
    u3 /= np.linalg.norm(u3)
    ub = (u1 + u3) / np.linalg.norm(u1 + u3)
    D = R_PIPE + G_A

    def meet(ua, da, ubb, db):
        return pc + np.linalg.solve(np.array([ua, ubb]), [da, db])

    J1 = meet(u1, D, ub, D_MID)
    J3 = meet(u3, D, ub, D_MID)

    def facet_end(u, other):
        t = np.array([-u[1], u[0]])
        if t @ other > 0:
            t = -t
        return pc + D * u + PAD_A * t

    E1 = facet_end(u1, u3)
    E3 = facet_end(u3, u1)

    def inward(p, r):
        v = p - pc
        return pc + r * v / np.linalg.norm(v)

    pts = [inward(E3, 26), E3, J3, J1, E1, inward(E1, 26)]
    sk = poly_face(Plane.XY.offset(H0), pts)
    return extrude(sk, amount=H1 - H0), pc + D_MID * ub


def web(crown):
    # top view: a band from the flange centre straight out, then to the saddle
    pc = PC[:2]
    toward = (crown - pc) / np.linalg.norm(crown - pc)
    deep = pc + (R_PIPE - 6) * toward          # end buried in the pipe
    path = [np.array([0.0, -14.0]), np.array([0.0, 38.0]), deep]
    hw = WEB_T / 2
    left, right = [], []
    for k, p in enumerate(path):
        tin = path[max(k, 1)] - path[max(k, 1) - 1]
        tin /= np.linalg.norm(tin)
        tout = path[min(k + 1, len(path) - 1)] - path[min(k + 1, len(path) - 1) - 1]
        tout /= np.linalg.norm(tout)
        t = (tin + tout) / np.linalg.norm(tin + tout)
        nn = np.array([-t[1], t[0]])
        s = 1.0 / (nn @ np.array([-tin[1], tin[0]]))     # mitre
        left.append(p + hw * s * nn)
        right.append(p - hw * s * nn)
    plan = extrude(poly_face(Plane.XY.offset(H0), left + right[::-1]),
                   amount=H1 - H0)

    # elevation (e, h), front edge kept inside the flange and its apron
    top_f = nc(OFF_B, SEAT_B / 2 - 1.8)
    low_f = nc(OFF_B, SEAT_LO + 0.6)
    f1, f2 = apron_feet()
    far = PC[1] + 10
    prof = [top_f, low_f, (f2[0] - 0.6, H0), (far, H0), (far, WEB_TOP)]
    elev = extrude(poly_face(Plane.YZ.offset(-60), prof), amount=120)
    return plan & elev


def apron_feet():
    s0 = nc(0, SEAT_LO)
    a = math.radians(APRON)
    dirv = math.sin(a) * NE - math.cos(a) * CE     # going down, away from B
    t = (H0 - s0[1]) / dirv[1]
    f1 = s0 + t * dirv
    f2 = f1 + np.array([1.6, 0.0])
    return f1, f2


def plate():
    # flange: build in a (-c1, c0, n) frame then place on B's face
    hi = SEAT_B / 2
    p = Pos(0, (hi + SEAT_LO) / 2, G_B / 2) * Box(SEAT_B, hi - SEAT_LO, G_B)
    p = fillet(p.edges().filter_by(Axis.Z).filter_by_position(Axis.Y, 0, 99),
               radius=3.0)
    pl = Plane(origin=(0, 0, 0), x_dir=(-1, 0, 0), z_dir=tuple(N))
    assert np.allclose(np.array(tuple(pl.y_dir)), C0, atol=1e-6)
    flange = pl.location * p

    # skirt under the flange's lower edge, down to the bed: its face towards
    # B (the apron) leaves B's face at APRON deg, so it never rests on B
    # outside the seat and is printable without support
    s0 = nc(0, SEAT_LO)
    f1, f2 = apron_feet()
    q2 = np.array([f2[0], (G_B - NE[0] * f2[0]) / NE[1]])   # on the back plane
    q3 = nc(G_B, SEAT_LO)
    skirt = extrude(poly_face(Plane.YZ.offset(-SEAT_B / 2),
                              [s0, f1, f2, q2, q3]), amount=SEAT_B)
    return flange + skirt


def build() -> Part:
    sad, crown = saddle()
    body = sad + web(crown) + plate()
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
    body = body.clean()
    world = Plane(origin=tuple(Bc), x_dir=tuple(c1), z_dir=tuple(d))
    return world.location * body


if __name__ == "__main__":
    part = build()
    print("solids", len(part.solids()), "volume", round(part.volume, 1))
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
