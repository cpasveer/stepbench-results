"""Connector bracket between round flange A and rectangular block B (FreeCAD).

Concept: two bolting plates (a truncated disc on A, a rectangle on B) joined by
a vertical dog-leg web. Every face is vertical or horizontal-ish so the part
prints standing upright, upside down on the flat top face (world z = Z_TOP),
without support: all bolt axes are horizontal, the web hangs from the bed, the
upper flanks of the A plate are cut at 50 deg and the B plate's seat edge above
the block is relieved with a steep chamfer. The web runs between the bolt rows
so every socket head can be reached and turned with an L-key.
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
V = App.Vector

Z_TOP = 37.0      # flat top face = print bed
T_A = 7.0         # plate thickness on A (grip)
T_B = 8.0         # plate thickness on B (grip)
W = 8.0           # web thickness
R_A = 36.0        # A plate radius (face 74.5 dia, keep >= 1 mm inside)
HOLE = {"M5": 5.5, "M6": 6.6}


class Frame:
    """origin O, normal n (horizontal), lateral u (horizontal), up z."""

    def __init__(self, face):
        self.O = V(*face["origin"])
        n = V(*face["normal"])
        n.z = 0
        self.n = n.normalize()
        self.u = V(-self.n.y, self.n.x, 0)
        self.z = V(0, 0, 1)

    def p(self, d, u, z):  # d along normal, u lateral, z world height
        return V(self.O.x, self.O.y, 0) + self.n * d + self.u * u + V(0, 0, z)

    def local(self, P):
        r = P - V(self.O.x, self.O.y, 0)
        return r.dot(self.n), r.dot(self.u), P.z


FA = Frame(FACES["A"])
FB = Frame(FACES["B"])


def prism(points, vec):
    w = Part.makePolygon(points + [points[0]])
    return Part.Face(w).extrude(vec)


def plate_a():
    zc = FA.O.z
    disc = Part.makeCylinder(R_A, T_A, FA.p(0, 0, zc), FA.n)
    # clip: flat top at Z_TOP, flanks at 50 deg from horizontal
    a = math.radians(50)
    nu, nv = math.sin(a), math.cos(a)
    c = 0.0
    for b in BOLTS:
        if b["size"] == "M5":
            _, u, z = FA.local(V(*b["position"]))
            if z > zc + 1:
                c = max(c, abs(u) * nu + (z - zc) * nv + b["head_diameter"] / 2 + 1.0)
    yt = Z_TOP - zc
    x1 = (c - nv * yt) / nu
    yb = -60.0
    xb = (c - nv * yb) / nu
    pts2 = [(-x1, yt), (x1, yt), (xb, yb), (-xb, yb)]
    pts = [FA.p(-1, x, zc + y) for x, y in pts2]
    clip = prism(pts, FA.n * (T_A + 2))
    return disc.common(clip)


def plate_b():
    hw = 46.0 / 2 - 1.0
    zb = FB.O.z - 96.0 / 2 + 1.5
    pts = [FB.p(0, -hw, zb), FB.p(0, hw, zb), FB.p(0, hw, Z_TOP), FB.p(0, -hw, Z_TOP)]
    pl = prism(pts, FB.n * T_B)
    # relieve the seat side above the block's top edge (steep chamfer, 63 deg)
    z0 = FB.O.z + 96.0 / 2 - 1.5
    k = 0.5
    wedge = [FB.p(-5, -40, z0), FB.p(0, -40, z0), FB.p(k * (Z_TOP + 10 - z0), -40, Z_TOP + 10),
             FB.p(-5, -40, Z_TOP + 10)]
    return pl.cut(prism(wedge, FB.u * 80))


def web():
    zlo, ztop = -120.0, Z_TOP
    h = ztop - zlo
    u0 = 3.0
    d1 = 42.0
    P0 = FA.p(T_A / 2, u0, zlo)
    P1 = FA.p(d1, u0, zlo)
    P2 = FB.p(T_B / 2, 0, zlo)

    def seg(Pa, Pb):
        t = Pb - Pa
        t.z = 0
        L = t.Length
        t.normalize()
        s = V(-t.y, t.x, 0) * (W / 2)
        pts = [Pa + s, Pb + s, Pb - s, Pa - s]
        return prism(pts, V(0, 0, h))

    w = seg(P0, P1).fuse(seg(P1, P2)).fuse(Part.makeCylinder(W / 2, h, P1, V(0, 0, 1)))
    # sloping underside: bottom rises toward A
    k = 0.8
    zA, dA = -14.0, 7.0

    def zb(d):
        return zA - k * (d - dA)

    ds = [-50.0, 150.0]
    pts = [FA.p(ds[0], -200, zb(ds[0])), FA.p(ds[1], -200, zb(ds[1])),
           FA.p(ds[1], -200, 200), FA.p(ds[0], -200, 200)]
    keep = prism(pts, FA.u * 400)
    return w.common(keep)


def holes():
    cut = None
    for b in BOLTS:
        P = V(*b["position"])
        ax = V(*b["axis"]).normalize()
        t = T_A if b["size"] == "M5" else T_B
        c = Part.makeCylinder(HOLE[b["size"]] / 2, t + 2, P - ax * 1, ax)
        cut = c if cut is None else cut.fuse(c)
    return cut


def build() -> Part.Shape:
    s = plate_a().fuse(plate_b()).fuse(web())
    s = s.cut(holes())
    s = s.removeSplitter()
    return s


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    print("solids", len(shape.Solids), "valid", shape.isValid(), "vol", shape.Volume)
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
