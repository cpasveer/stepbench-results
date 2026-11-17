"""facing-L3-s1 connector, FreeCAD.

Two flanges (A: square, B: round-ish) joined by a hollow, kite-shaped column.
The part is designed to print lying on its side: the print bed is the plane
1 mm inside A's square edge (local +y side of A), so the flange plates stand
up as walls, all bolt bearing faces are vertical, and every downward-facing
side face is at least 50 deg from horizontal (no supports).
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
PLC = TASK["poses"][0]["placements"]
FACES = POSE["mounting_faces"]
BOLTS = POSE["bolts"]
V = App.Vector

# ---- frame: A-local x/y in the horizontal plane, origin at A face centre
cA = V(*FACES[0]["origin"])
cB = V(*FACES[1]["origin"])
R = PLC["A"]["rotation"]
LX = V(R[0][0], R[1][0], 0)
LY = V(R[0][1], R[1][1], 0)
ZA = cA.z          # A mounting face (part sits above)
ZB = cB.z          # B mounting face (part sits below)

T_A = 4.0          # A flange thickness (grip 3-25)
T_B = 4.5          # B flange thickness (grip 3.75-25)
WALL = 2.0         # column wall
BED = 34.5         # local y of print bed (A edge 36.5, seat limit 35.5)
BETA = math.radians(50)   # minimum steepness of down-facing walls


def loc(p):
    d = V(p[0], p[1], 0) - V(cA.x, cA.y, 0)
    return (d.dot(LX), d.dot(LY))


def W(x, y, z):
    """local (x, y) + world z -> world vector"""
    return V(cA.x, cA.y, 0) + LX * x + LY * y + V(0, 0, z)


A_B = [(b, loc(b["position"])) for b in BOLTS if b["id"][0] == "A"]
B_B = [(b, loc(b["position"])) for b in BOLTS if b["id"][0] == "B"]
CB = loc(cB)


# ---- 2D convex hull of circles (r may be 0), returned as exact lines + arcs
def _hull_pts(pts):
    pts = sorted(set(pts))
    def cr(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 1e-12:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 1e-12:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def circle_hull_wire(circles, z):
    """circles: list of ((x, y), r). Returns a closed wire at height z (CCW)."""
    samples = {}
    n = 720
    for i, (c, r) in enumerate(circles):
        if r == 0:
            samples[(round(c[0], 9), round(c[1], 9))] = i
        else:
            for k in range(n):
                a = 2 * math.pi * k / n
                samples[(round(c[0] + r*math.cos(a), 9), round(c[1] + r*math.sin(a), 9))] = i
    h = _hull_pts(list(samples))
    order = []
    for p in h:
        i = samples[p]
        if not order or order[-1] != i:
            order.append(i)
    while len(order) > 1 and order[0] == order[-1]:
        order.pop()
    m = len(order)
    tan_out, tan_in = {}, {}
    for k in range(m):
        i, j = order[k], order[(k+1) % m]
        (c1, r1), (c2, r2) = circles[i], circles[j]
        dx, dy = c2[0]-c1[0], c2[1]-c1[1]
        L = math.hypot(dx, dy)
        ux, uy = dx/L, dy/L
        s = (r1 - r2) / L
        q = math.sqrt(max(0.0, 1 - s*s))
        nx, ny = s*ux + q*uy, s*uy - q*ux     # normal to the right of travel
        tan_out[k] = (c1[0] + r1*nx, c1[1] + r1*ny)
        tan_in[(k+1) % m] = (c2[0] + r2*nx, c2[1] + r2*ny)
    edges = []
    for k in range(m):
        c, r = circles[order[k]]
        a, b = tan_in[k], tan_out[k]
        if r > 0 and math.hypot(a[0]-b[0], a[1]-b[1]) > 1e-6:
            a0 = math.atan2(a[1]-c[1], a[0]-c[0])
            a1 = math.atan2(b[1]-c[1], b[0]-c[0])
            # hull is clockwise from the sampling? determine sweep sense
            am = a0 + ((a1 - a0) % (2*math.pi)) / 2
            pm = (c[0] + r*math.cos(am), c[1] + r*math.sin(am))
            edges.append(Part.Arc(W(a[0], a[1], z), W(pm[0], pm[1], z), W(b[0], b[1], z)).toShape())
        nxt = tan_in[(k+1) % m]
        edges.append(Part.makeLine(W(b[0], b[1], z), W(nxt[0], nxt[1], z)))
    return Part.Wire(Part.__sortEdges__(edges))


def poly_wire(pts, z):
    vs = [W(x, y, z) for x, y in pts]
    return Part.makePolygon(vs + [vs[0]])


def prism(wire, h):
    return Part.Face(wire).extrude(V(0, 0, h))


# ---- column cross-section (local): flat foot on the bed, 50 deg flanks, top apex
XB = -3.0          # column centre line x
FOOT = 3.0         # half width of the foot
YT = -22.0         # apex (clear of B1 head)
a = math.tan(BETA)
YS = ((BED - YT) - 0.0 + a*FOOT) / 2 + YT  # side-vertex height (equal flank slopes)
YS = (YT + BED + a*FOOT) / 2
HW = (YS - YT) / a                           # half width at side vertices
COL = [(XB - FOOT, BED), (XB + FOOT, BED), (XB + HW, YS), (XB, YT), (XB - HW, YS)]


def inner_offset(poly, t):
    """inward offset of a convex CCW polygon by t (sharp corners)."""
    n = len(poly)
    lines = []
    for i in range(n):
        p, q = poly[i], poly[(i+1) % n]
        dx, dy = q[0]-p[0], q[1]-p[1]
        L = math.hypot(dx, dy)
        nx, ny = -dy/L, dx/L                 # left normal = inward for CCW
        lines.append(((p[0]+t*nx, p[1]+t*ny), (dx, dy)))
    out = []
    for i in range(n):
        (p1, d1), (p2, d2) = lines[i-1], lines[i]
        den = d1[0]*d2[1] - d1[1]*d2[0]
        s = ((p2[0]-p1[0])*d2[1] - (p2[1]-p1[1])*d2[0]) / den
        out.append((p1[0]+s*d1[0], p1[1]+s*d1[1]))
    return out


def ccw(poly):
    s = sum(poly[i][0]*poly[(i+1) % len(poly)][1] - poly[(i+1) % len(poly)][0]*poly[i][1]
            for i in range(len(poly)))
    return poly if s > 0 else poly[::-1]


def build() -> Part.Shape:
    col = ccw(COL)
    hole = inner_offset(col, WALL)

    # A flange: rounded at the two upper bolts, square on the bed side
    ra = 4.5
    circA = [(p, ra) for _, p in A_B if p[1] < 0]
    circA += [((-BED, BED), 0), ((BED, BED), 0)]
    fa = prism(circle_hull_wire(circA, ZA), T_A)

    # B flange: hull of bolt bosses, column foot and a foot on the bed
    rb = 5.25
    circB = [(p, rb) for _, p in B_B] + [(c, 0) for c in col]
    circB += [((-33.0, BED), 0), ((24.0, BED), 0)]   # >= 50 deg flanks to B2/B3 bosses
    fb = prism(circle_hull_wire(circB, ZB - T_B), T_B)

    colsolid = prism(poly_wire(col, ZA + T_A), (ZB - T_B) - (ZA + T_A))
    body = fa.fuse([fb, colsolid])
    # through opening of the hollow column
    body = body.cut(prism(poly_wire(hole, ZA - 1), ZB - ZA + 2))
    # bolt holes
    for b, p in A_B:
        body = body.cut(Part.makeCylinder(4.5/2, 50, W(p[0], p[1], ZA - 1), V(0, 0, 1)))
    for b, p in B_B:
        body = body.cut(Part.makeCylinder(5.5/2, 50, W(p[0], p[1], ZB + 1), V(0, 0, -1)))
    return body.removeSplitter()


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", shape.Volume, "valid", shape.isValid(), "solids", len(shape.Solids))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
