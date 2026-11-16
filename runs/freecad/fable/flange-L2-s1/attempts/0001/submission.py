"""Connector for flange-L2-s1: a closed box frame, printed standing on its
bottom (world Z up, both mounting faces upright).

In plan the part is a ring of four upright walls: a wall on the round flange A,
a wall on the block B, and two side webs outside the bolt patterns, so every
bolt head stays open to the hollow in the middle and the hex keys swing there.
Each wall touches its fixed part only through a raised seat boss (a round boss
with a 50 degree flared edge on A, a rectangular pad on B), so the frame can be
wider than the mounting faces without lying on them.  Every wall runs straight
down to the bed, so nothing overhangs.

    freecadcmd submission.py             # writes output.step
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
POSE = TASK["poses"][0]
FACES = POSE["resolved"]["mounting_faces"]
BOLTS = POSE["resolved"]["bolts"]
V = App.Vector
Z = V(0, 0, 1)

# ---- frames -------------------------------------------------------------
A0 = V(*FACES[0]["origin"])
NA = V(*FACES[0]["normal"])
UA = Z.cross(NA)                      # horizontal, in A's face
B0 = V(*FACES[1]["origin"])
NB = V(*FACES[1]["normal"])
rotB = POSE["placements"]["B"]["rotation"]
XB = V(rotB[0][0], rotB[1][0], rotB[2][0])   # B's short direction, horizontal

# ---- dimensions ---------------------------------------------------------
R_FLANGE = 74.5 / 2
B_W, B_H = 46.0, 96.0
SETBACK = 1.0
T_WALL = 7.0          # grip under every bolt head (wall + seat boss)
C_A = 1.2             # height of the round seat boss on A
C_B = 1.5             # height of the seat pad on B
R_SEAT = 35.0         # seat radius on A (flange 37.25)
R_FLARE = 36.4        # boss radius where it meets the wall: 49 deg from level
T_WEB = 5.0
U_IN = 38.0           # inner faces of the side webs, from A's axis
U_OUT = U_IN + T_WEB
S_KNEE = 40.0         # where the far web turns towards B (beyond A's keys)
XB_SEAT = B_W / 2 - SETBACK           # 22
Z_BOT = B0.z - B_H / 2 + SETBACK      # bed level, bottom of B's seat
Z_TOP = B0.z + B_H / 2 - SETBACK      # top of B's seat, of the webs
Z_ATOP = A0.z + R_FLANGE              # top of the wall on A
D_A = 5.5             # M5 clearance
D_B = 6.6             # M6 clearance
R_IN_A, R_IN_B, R_KNEE = 3.0, 1.5, 5.0


def pa(s, u, z=Z_BOT):
    """Point given in A's frame: s out of the face, u across, world z."""
    return V(A0.x, A0.y, 0) + NA * s + UA * u + Z * z


def pb(x, d, z=Z_BOT):
    """Point given in B's frame: x across, d out of the face, world z."""
    return V(B0.x, B0.y, 0) + XB * x + NB * d + Z * z


def to_a(p):
    q = p - V(A0.x, A0.y, 0)
    return (q.dot(NA), q.dot(UA))


def meet(p, dp, q, dq):
    """Intersection of two lines in the plan (2D tuples)."""
    det = dp[0] * (-dq[1]) + dq[0] * dp[1]
    t = ((q[0] - p[0]) * (-dq[1]) + dq[0] * (q[1] - p[1])) / det
    return (p[0] + t * dp[0], p[1] + t * dp[1])


def prism(pts_su, z0, z1):
    pts = [pa(s, u, z0) for s, u in pts_su]
    wire = Part.makePolygon(pts + [pts[0]])
    return Part.Face(wire).extrude(Z * (z1 - z0))


def fillet_vertical(solid, corners):
    """Round the upright edges standing on the given plan points."""
    for (s, u), r in corners:
        target = pa(s, u, Z_BOT)
        for e in solid.Edges:
            a, b = e.Vertexes[0].Point, e.Vertexes[-1].Point
            if len(e.Vertexes) == 2 and abs(a.x - b.x) < 1e-6 and abs(a.y - b.y) < 1e-6 \
                    and math.hypot(a.x - target.x, a.y - target.y) < 1e-4:
                solid = solid.makeFillet(r, [e])
                break
        else:
            raise RuntimeError("no upright edge at %r" % ((s, u),))
    return solid


def build() -> Part.Shape:
    xb = (XB.dot(NA), XB.dot(UA))     # B's across direction, in A's frame
    b_back = to_a(pb(0, T_WALL))      # a point on B wall's inner face
    b_front = to_a(pb(0, C_B))        # a point on B wall's outer face

    # hollow (inner outline), in A's frame
    i1 = (T_WALL, -U_IN)
    i2 = (T_WALL, U_IN)
    i3 = meet((T_WALL, U_IN), (1, 0), b_back, xb)
    i4 = to_a(pb(XB_SEAT, T_WALL))
    i5 = (S_KNEE, -U_IN)
    # outer outline
    dg = (i4[0] - i5[0], i4[1] - i5[1])
    ln = math.hypot(*dg)
    dg = (dg[0] / ln, dg[1] / ln)
    off = (i5[0] + T_WEB * dg[1], i5[1] - T_WEB * dg[0])
    o1 = (C_A, -U_OUT)
    o2 = (C_A, U_OUT)
    o3 = meet((C_A, U_OUT), (1, 0), b_front, xb)
    o4 = meet(off, dg, b_front, xb)
    o5 = meet(off, dg, (C_A, -U_OUT), (1, 0))

    outer = prism([o1, o2, o3, o4, o5], Z_BOT, Z_TOP)
    outer = fillet_vertical(outer, [(o5, R_KNEE + T_WEB)])
    inner = prism([i1, i2, i3, i4, i5], Z_BOT - 1, Z_TOP + 1)
    inner = fillet_vertical(inner, [(i1, R_IN_A), (i2, R_IN_A), (i3, R_IN_B),
                                    (i4, R_IN_B), (i5, R_KNEE)])
    part = outer.cut(inner)

    # wall on A: full width up to the web tops, rounded shoulders above
    r_sh = Z_ATOP - Z_TOP
    w = U_OUT
    p = lambda u, z: pa(C_A, u, z)
    edges = [
        Part.makePolygon([p(w, Z_TOP), p(w, Z_BOT), p(-w, Z_BOT), p(-w, Z_TOP)]),
        Part.Arc(p(-w, Z_TOP),
                 p(-w + r_sh * (1 - math.sqrt(0.5)), Z_TOP + r_sh * math.sqrt(0.5)),
                 p(-w + r_sh, Z_ATOP)).toShape(),
        Part.makeLine(p(-w + r_sh, Z_ATOP), p(w - r_sh, Z_ATOP)),
        Part.Arc(p(w - r_sh, Z_ATOP),
                 p(w - r_sh * (1 - math.sqrt(0.5)), Z_TOP + r_sh * math.sqrt(0.5)),
                 p(w, Z_TOP)).toShape(),
    ]
    wire = Part.Wire([e for sh in edges for e in sh.Edges])
    wall_a = Part.Face(wire).extrude(NA * (T_WALL - C_A))
    part = part.fuse(wall_a)

    # seat boss on A: flared so its lower half prints without support
    boss_a = Part.makeCone(R_SEAT, R_FLARE, C_A, A0, NA)
    part = part.fuse(boss_a)

    # seat pad on B
    pts = [pb(-XB_SEAT, 0), pb(XB_SEAT, 0), pb(XB_SEAT, C_B), pb(-XB_SEAT, C_B)]
    pad_b = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(Z * (Z_TOP - Z_BOT))
    part = part.fuse(pad_b)
    part = part.removeSplitter()

    # clearance holes
    for b in BOLTS:
        d = D_A if b["size"] == "M5" else D_B
        ax = V(*b["axis"])
        pos = V(*b["position"])
        part = part.cut(Part.makeCylinder(d / 2, T_WALL + 2, pos - ax, ax))
    return part.removeSplitter()


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) != 1 or not shape.isValid():
        raise RuntimeError("not one valid solid: %d" % len(shape.Solids))
    shape = shape.Solids[0]
    shape.exportStep(OUT)
    print("wrote", OUT, "volume %.0f mm3" % shape.Volume, "faces", len(shape.Faces))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
