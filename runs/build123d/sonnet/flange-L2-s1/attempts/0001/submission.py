"""Bracket joining flange mount A and block mount B.

Two pads (A: tombstone with a foot, B: rectangle) seated on the mounting faces,
joined by a vertical side wall on the +u side, clear of every bolt head and key
swing.  All outer walls are prismatic about the vertical axis so the part prints
upright on its flat foot without support.
"""
import json
import math
import pathlib

from build123d import (BuildPart, BuildSketch, Cylinder, Locations, Plane,
                       Polygon, Axis, Pos, Part, Vector, export_step, extrude,
                       Mode, Rot)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

T = 7.0          # pad thickness (grip)
BOSS = 1.0       # seat boss height on the face
Z_BED = -64.8    # underside of everything (B face bottom + 1.2 inset)
Z_WALL_TOP = 21.65


def frame(name):
    f = FACES[name]
    o = Vector(*f["origin"])
    n = Vector(*f["normal"])
    u = Vector(-n.Y, n.X, 0).normalized()
    return Plane(origin=o, x_dir=u, z_dir=n)  # y_dir = up


PA, PB = frame("A"), frame("B")


def prism(plane, pts, s0, s1):
    with BuildPart() as p:
        with BuildSketch(plane.offset(s0)):
            Polygon(*pts, align=None)
        extrude(amount=s1 - s0)
    return p.part


def build() -> Part:
    zA = FACES["A"]["origin"][2]
    zB = FACES["B"]["origin"][2]
    a = 39.0
    t = a * math.tan(math.radians(22.5))
    zb = Z_BED - zA
    zt = Z_WALL_TOP - zA
    uw = 48.0
    # A pad outline (u, y): octagon round the bolt circle + foot + wall root
    ptsA = [(-t, a), (-a, t), (-a, -t), (-t, -a), (-t, zb), (uw, zb),
            (uw, zt), (a, zt), (a, t)]
    # keep it simple/convex-ish: (a,t)->(t,a) closes via polygon
    ptsA = [(-t, a), (-a, t), (-a, -t), (-t, -a), (-t, zb), (uw, zb),
            (uw, zt), (a, t)]
    padA = prism(PA, ptsA, 1.0, T)
    bossA = Cylinder(36.0, BOSS + 0.5, align=(__import__("build123d").Align.CENTER,) * 2 + (__import__("build123d").Align.MIN,))
    bossA = PA.location * bossA
    # B pad: rectangle inset 1.2 from the 46x96 face, plus a wing toward the wall
    yb0, yb1 = Z_BED - zB, 28.8 - zB
    padB = prism(PB, [(-21.8, yb0), (21.8, yb0), (21.8, yb1), (-21.8, yb1)], 0.0, T)
    wingB = prism(PB, [(-34.0, yb0), (-20.0, yb0), (-20.0, Z_WALL_TOP - zB),
                       (-34.0, Z_WALL_TOP - zB)], 1.0, T)
    # side wall: vertical prism in plan
    import numpy as np
    A0 = np.array(FACES["A"]["origin"][:2]); B0 = np.array(FACES["B"]["origin"][:2])
    nA = np.array(FACES["A"]["normal"][:2]); nB = np.array(FACES["B"]["normal"][:2])
    uA = np.array([-nA[1], nA[0]])

    def wallpt(u, sB):
        s = (sB - (((A0 - B0) + u * uA) @ nB)) / (nA @ nB)
        return A0 + u * uA + s * nA
    u0, u1 = uw - 7.0, uw
    pts = [A0 + u0 * uA + 4 * nA, A0 + u1 * uA + 4 * nA, wallpt(u1, 4), wallpt(u0, 4)]
    with BuildPart() as w:
        with BuildSketch(Plane.XY.offset(Z_BED)):
            Polygon(*[(float(p[0]), float(p[1])) for p in pts], align=None)
        extrude(amount=Z_WALL_TOP - Z_BED)
    wall = w.part

    body = padA + bossA + padB + wingB + wall
    # bolt holes
    for b in BOLTS:
        d = 5.5 if b["size"] == "M5" else 6.6
        pos = Vector(*b["position"]); ax = Vector(*b["axis"])
        cyl = Cylinder(d / 2, T + 4, align=(__import__("build123d").Align.CENTER,) * 3)
        pl = Plane(origin=pos + ax * (T / 2), z_dir=ax)
        body = body - (pl.location * cyl)
    return body.clean()


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", part.volume, part.is_valid)
