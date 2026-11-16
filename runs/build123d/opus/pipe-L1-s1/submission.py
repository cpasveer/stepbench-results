"""Pipe-to-flange bracket for task pipe-L1-s1 (build123d).

The part is three members fused into one solid:

* a saddle: a cylindrical shell sector that wraps the 68 mm pipe over
  +/-52 deg and 60 mm of its length, bored to the pipe radius so it bears on
  the round surface, with spot-faced counterbores for the four M5 screws;
* a flange plate, 34 x 34 x 8 mm, that sits 2 mm inside the 38 mm square
  mounting face of B and carries the four M4 screws;
* a single flat web, 10 mm thick, that joins the two.  It lies in a plane
  that contains the pipe axis direction (so it meets the saddle along a
  straight axial line between the two bolt rows) and passes through the
  middle of the flange between its bolt rows, leaving every screw head and
  hex key free to swing outward.
"""
import json
import math
import pathlib

import numpy as np
from build123d import (
    Axis,
    Box,
    Cylinder,
    Location,
    Part,
    Plane,
    Polyline,
    Pos,
    Rot,
    Vector,
    Line,
    ThreePointArc,
    export_step,
    extrude,
    fillet,
    make_face,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
PL = TASK["poses"][0]["placements"]
BOLTS = {b["id"]: b for b in TASK["poses"][0]["resolved"]["bolts"]}

RA = np.array(PL["A"]["rotation"])
OA = np.array(PL["A"]["origin"])
RB = np.array(PL["B"]["rotation"])
OB = np.array(PL["B"]["origin"])

R_PIPE = 34.0
# design parameters
SADDLE_T = 9.0          # saddle wall thickness
SADDLE_HALF_LEN = 30.0  # along pipe axis
SADDLE_HALF_ANG = 52.0  # deg either side of the web
A_GRIP = 7.0            # saddle thickness under the M5 heads
A_HOLE = 5.5
A_CBORE = 11.0
PLATE = 34.0            # 38 mm face, 2 mm set back all round
PLATE_T = 8.0
B_HOLE = 4.5
WEB_T = 10.0
WEB_Y_ON_PLATE = 3.0    # web offset on the flange (B local y)
WEB_SAG = 8.0           # depth of the concave flanks of the web
WEB_THETA = -12.0       # web angle on the saddle (deg, A local)


def v(a):
    return Vector(*[float(x) for x in a])


def plane_A():
    return Plane(origin=v(OA), x_dir=v(RA[:, 0]), z_dir=v(RA[:, 2]))


def plane_B():
    return Plane(origin=v(OB), x_dir=v(RB[:, 0]), z_dir=v(RB[:, 2]))


def a_local(p):
    return OA + RA @ np.asarray(p, float)


def b_local(p):
    return OB + RB @ np.asarray(p, float)


def saddle() -> Part:
    ro = R_PIPE + SADDLE_T
    L = 2 * SADDLE_HALF_LEN
    shell = Rot(0, 90, 0) * Cylinder(ro, L)  # axis along local x
    # angular window centred on the web angle
    c = math.radians(WEB_THETA)
    h = math.radians(SADDLE_HALF_ANG)
    pts = [(0, 0)]
    n = 8
    for i in range(n + 1):
        t = c - h + 2 * h * i / n
        rr = ro * 1.6
        pts.append((rr * math.sin(t), rr * math.cos(t)))
    wedge_face = make_face(Polyline(*[(-L, y, z) for y, z in pts], close=True))
    # extrude along +x from x=-L by 2L -> covers the shell
    wedge = extrude(wedge_face, amount=2 * L, dir=(1, 0, 0))
    s = shell & wedge
    # round the four corners (edges running radially at the ends)
    corner_edges = [
        e
        for e in s.edges()
        if abs(abs(e.center().X) - SADDLE_HALF_LEN) < 1e-3
        and e.geom_type == "LINE"
        and abs(Vector(e.tangent_at(0.5)).X) < 1e-3
    ]
    if corner_edges:
        s = fillet(corner_edges, 6.0)
    return plane_A().location * s


def plate() -> Part:
    p = Pos(0, 0, PLATE_T / 2) * Box(PLATE, PLATE, PLATE_T)
    p = fillet(p.edges().filter_by(Axis.Z), 5.0)
    return plane_B().location * p


def web() -> Part:
    d = RA[:, 0]
    th = math.radians(WEB_THETA)
    r_s = R_PIPE + SADDLE_T / 2
    Ps = a_local((0, r_s * math.sin(th), r_s * math.cos(th)))
    z_in = PLATE_T - 2.0
    Pp = b_local((0, WEB_Y_ON_PLATE, z_in))
    m = np.cross(d, Ps - Pp)
    m /= np.linalg.norm(m)

    def plate_pt(x):
        # point on plane z_B = z_in with B-local x, lying in the web plane
        p0 = b_local((x, 0, z_in))
        by = RB[:, 1]
        y = -np.dot(p0 - Pp, m) / np.dot(by, m)
        return p0 + y * by

    def saddle_pt(a):
        return a_local((a, r_s * math.sin(th), r_s * math.cos(th)))

    p0, p1 = plate_pt(-12.0), plate_pt(12.0)
    s1, s0 = saddle_pt(26.0), saddle_pt(-26.0)
    centre = (p0 + p1 + s0 + s1) / 4

    def flank(a, b, sag):
        # concave arc between the flange and the saddle: the web is waisted,
        # so it leaves both members steeply instead of at a shallow angle
        mid = (a + b) / 2
        inward = centre - mid
        inward -= np.dot(inward, b - a) / np.dot(b - a, b - a) * (b - a)
        inward /= np.linalg.norm(inward)
        return ThreePointArc(v(a), v(mid + sag * inward), v(b))

    edges = [
        Line(v(p0), v(p1)),
        flank(p1, s1, WEB_SAG),
        Line(v(s1), v(s0)),
        flank(s0, p0, WEB_SAG),
    ]
    face = make_face(edges)
    w = extrude(face, amount=WEB_T / 2, dir=v(m), both=True)
    return w


def pipe_cutter() -> Part:
    return plane_A().location * (Rot(0, 90, 0) * Cylinder(R_PIPE, 400))


def bolt_cuts() -> Part:
    cuts = []
    for bid, b in BOLTS.items():
        p = np.array(b["position"])
        ax = np.array(b["axis"])
        if b["size"] == "M5":
            hole_d, depth = A_HOLE, 60.0
            pl = Plane(origin=v(p - 5 * ax), z_dir=v(ax))
            cuts.append(pl.location * Pos(0, 0, depth / 2) * Cylinder(hole_d / 2, depth))
            pc = Plane(origin=v(p + A_GRIP * ax), z_dir=v(ax))
            cuts.append(pc.location * Pos(0, 0, 20) * Cylinder(A_CBORE / 2, 40))
        else:
            pl = Plane(origin=v(p - 5 * ax), z_dir=v(ax))
            pl = Plane(origin=v(p - 1 * ax), z_dir=v(ax))
            cuts.append(pl.location * Pos(0, 0, (PLATE_T + 2) / 2) * Cylinder(B_HOLE / 2, PLATE_T + 2))
    out = cuts[0]
    for c in cuts[1:]:
        out = out + c
    return out


def build() -> Part:
    body = saddle() + plate() + web()
    body = body - pipe_cutter()
    body = body - bolt_cuts()
    return body


if __name__ == "__main__":
    part = build()
    sol = part.solids()
    print("solids", len(sol), "valid", part.is_valid, "volume", round(part.volume, 1))
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
