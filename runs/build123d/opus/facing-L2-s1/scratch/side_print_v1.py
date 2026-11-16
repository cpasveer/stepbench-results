"""Connector for facing-L2-s1: square M4 mount A (below) to round M5 mount B (above).

Concept
-------
Two 6 mm flanges joined by a hollow kite-section tube, designed to be printed
lying on its side (bed plane = one side face of the A flange, local t = +35),
so that every bolt seat is a vertical wall in print, layers run along the
load path between the two flanges, and no support is needed:

* A flange: 70 x 70 square (1 mm+ inside the 73 mm mount face), rounded on the
  top-of-print corners, steep chamfers on the bed corners.
* B flange: 39.9 mm radius disc (inside the 84.5 mm face) that runs into the
  bed with 50-degree tangent flanks (a truncated teardrop), so nothing
  overhangs more than 40 degrees.
* Tube: kite section (flat on the bed, 50-degree flanks, pointed roof) with a
  4 mm wall, open through both flanges, filleted into both flanges.

All geometry is built in the A-mount frame (s, t, z) and then placed in world.
"""
import json
import math
import pathlib

from build123d import (
    Align, Axis, Box, Circle, Cylinder, Location, Part, Plane, Polygon, Pos,
    Rectangle, Rot, Sketch, export_step, extrude, fillet, offset, Kind,
    make_face, Polyline, Vector,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
FACES = POSE["resolved"]["mounting_faces"]
BOLTS = POSE["resolved"]["bolts"]

A0 = Vector(*POSE["placements"]["A"]["origin"])
RA = POSE["placements"]["A"]["rotation"]
U = Vector(RA[0][0], RA[1][0], 0)  # A local x in world
V = Vector(RA[0][1], RA[1][1], 0)  # A local y in world
ANG = math.degrees(math.atan2(U.Y, U.X))

ZA = FACES[0]["origin"][2]  # A face (top of A), -94
ZB = FACES[1]["origin"][2]  # B face (bottom of B), -25.5
T_PLATE = 6.0
HALF_A = 35.0           # A flange half size (face is 36.5)
BED = HALF_A            # print bed plane: t = +35
R_B = 39.9              # B flange radius (face radius 42.25, limit 41.25)
FLANK = math.radians(50)  # flank inclination from the bed plane
WALL = 4.0


def to_local(p):
    d = Vector(p[0], p[1], 0) - Vector(A0.X, A0.Y, 0)
    return d.dot(U), d.dot(V)


BC = to_local(FACES[1]["origin"])  # B centre in local frame


def bolt_local():
    return [(b, *to_local(b["position"])) for b in BOLTS]


def a_flange() -> Part:
    h = HALF_A
    # bed side (t=+h) corners: chamfer 3 along bed, 4.5 up the side (56 deg)
    pts = [(-h + 3, h), (h - 3, h), (h, h - 4.5), (h, -h), (-h, -h), (-h, h - 4.5)]
    sk = Sketch() + Polygon(*pts, align=None)
    # round the two top-of-print corners (t = -h)
    verts = sk.vertices().filter_by(lambda v: abs(v.Y + h) < 1e-6)
    sk = fillet(verts, 6.0)
    return Pos(0, 0, ZA) * extrude(sk, T_PLATE, dir=(0, 0, 1))


def b_flange() -> Part:
    sc, tc = BC
    d = BED - tc                      # centre to bed distance
    ns, nc = math.sin(FLANK), math.cos(FLANK)
    tan_r = (sc + R_B * ns, tc + R_B * nc)
    tan_l = (sc - R_B * ns, tc + R_B * nc)
    lam = (d - R_B * nc) / ns
    bot = R_B * ns - lam * nc
    trap = Polygon(tan_l, tan_r, (sc + bot, BED), (sc - bot, BED), align=None)
    sk = Sketch() + [Pos(sc, tc) * Circle(R_B), trap]
    sk = sk & (Pos(0, BED - 100) * Rectangle(400, 200))
    return Pos(0, 0, ZB - T_PLATE) * extrude(sk, T_PLATE, dir=(0, 0, 1))


S0 = (BC[0]) / 2.0  # tube centre line between the two mount centres


def kite(grow=0.0):
    w0, w1, t1, top = 6.0, 16.0, 23.0, -12.0
    pts = [(S0 - w0, BED), (S0 + w0, BED), (S0 + w1, t1), (S0, top), (S0 - w1, t1)]
    sk = Sketch() + Polygon(*pts, align=None)
    up = sk.vertices().filter_by(lambda v: v.Y < BED - 1e-3)
    return fillet(up, 4.0)


def build() -> Part:
    zlo, zhi = ZA + T_PLATE, ZB - T_PLATE
    outer = kite()
    tube = Pos(0, 0, zlo) * extrude(outer, zhi - zlo, dir=(0, 0, 1))
    part = a_flange() + b_flange() + tube
    part = part.clean()

    # fillet tube into both flanges (junction edges off the bed plane)
    def junction(e):
        c = e.center()
        on_plate = abs(c.Z - zlo) < 1e-4 or abs(c.Z - zhi) < 1e-4
        return on_plate and abs(c.Y - BED) > 1e-3 and e.length > 1.0

    tube_faces = [f for f in part.faces() if abs(f.normal_at().Z) < 1e-6
                  and zlo - 1e-4 <= f.bounding_box().min.Z
                  and f.bounding_box().max.Z <= zhi + 1e-4]
    edges = []
    for f in tube_faces:
        for e in f.edges():
            if junction(e):
                edges.append(e)
    part = part.fillet(5.0, edges)

    # bore of the tube, through both flanges
    inner = offset(outer, -WALL, kind=Kind.ARC)
    bore = Pos(0, 0, ZA - 1) * extrude(inner, ZB - ZA + 2, dir=(0, 0, 1))
    part = part - bore

    # bolt holes
    for b, s, t in bolt_local():
        if b["size"] == "M4":
            z0, d = ZA - 1, 4.5
        else:
            z0, d = ZB - T_PLATE - 1, 5.5
        part -= Pos(s, t, z0) * Cylinder(d / 2, T_PLATE + 2,
                                         align=(Align.CENTER, Align.CENTER, Align.MIN))

    part = part.clean()
    return Pos(A0.X, A0.Y, 0) * Rot(0, 0, ANG) * part


if __name__ == "__main__":
    p = build()
    print("valid", p.is_valid, "solids", len(p.solids()), "vol", round(p.volume))
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
