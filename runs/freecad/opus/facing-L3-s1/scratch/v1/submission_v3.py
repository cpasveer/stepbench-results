"""facing-L3-s1 connector, modelled in FreeCAD (Part workbench, CSG).

Printed upright: A's seat face on the bed, B's seat face on top.

  * A flange: rounded square plate around A's 4x M4 holes.
  * A short lofted base cone (centred on A, narrowing upward) stiffens the
    A plate and carries the load into a round column centred on B.
  * The column flares out at 50 deg into B's round seat ring, so B's
    bolts sit right on the rim of a stiff cone.
  * Each B bolt head sits in a pocket cut up into the cone; the pocket
    ceiling (the head's bearing face) is a short 10 mm bridge held by the
    pocket walls on both sides, and the pocket opens outward so the hex
    key can swing.
  * The column is hollow, open through both seats; every inner face looks
    upward, so nothing needs support.
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
FACES = POSE["mounting_faces"]
BOLTS = POSE["bolts"]
V = App.Vector
Z = V(0, 0, 1)

cA = V(*FACES[0]["origin"])          # A seat, part above it
cB = V(*FACES[1]["origin"])          # B seat, part below it
ZA, ZB = cA.z, cB.z
A_BOLTS = [b for b in BOLTS if b["id"].startswith("A")]
B_BOLTS = [b for b in BOLTS if b["id"].startswith("B")]

T_A = 4.0            # A plate (grip 3..25)
T_B = 4.5            # B ring  (grip 3.75..25)
R_AFL = 4.5          # A plate corner radius around each bolt
HALF_A = 34.5        # A plate half width (face is 73, keep 1 mm in)
R_B_OUT = 41.0       # B seat outer radius (face is 84.5 dia, keep 1 mm in)
R_COL = 26.0         # column outer radius (centred on B)
R_HOLE = 22.0        # column bore
FLARE = math.radians(50)   # all flank faces 50 deg from horizontal (>45 for FDM)
POCKET_A = 4.5       # A head pocket radius (head 7 + clearance)
POCKET_B = 5.0       # B head pocket radius (head 8.5 + clearance)
D_A, D_B = 4.5, 5.5  # clearance holes
R_A = TASK["poses"][0]["placements"]["A"]["rotation"]
LX = V(R_A[0][0], R_A[1][0], 0)
LY = V(R_A[0][1], R_A[1][1], 0)


def xy(p, z):
    return V(p.x, p.y, z)


def hull_wire(centres, r, z):
    """outline of the convex hull of equal circles (exact lines + arcs)."""
    c = [xy(p, z) for p in centres]
    m = sum(c, V()) * (1.0 / len(c))
    c.sort(key=lambda p: math.atan2(p.y - m.y, p.x - m.x))
    n = len(c)
    edges = []
    for i in range(n):
        p, q = c[i], c[(i + 1) % n]
        d = q - p
        d.normalize()
        out = V(d.y, -d.x, 0)
        edges.append(Part.makeLine(p + out * r, q + out * r))
    for i in range(n):
        p, q, s = c[i - 1], c[i], c[(i + 1) % n]
        d1 = q - p; d1.normalize()
        d2 = s - q; d2.normalize()
        o1 = V(d1.y, -d1.x, 0); o2 = V(d2.y, -d2.x, 0)
        mid = o1 + o2; mid.normalize()
        edges.append(Part.Arc(q + o1 * r, q + mid * r, q + o2 * r).toShape())
    return Part.Wire(Part.__sortEdges__(edges))


def square_wire(half, z):
    c = xy(cA, z)
    pts = [c + LX * sx * half + LY * sy * half for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return Part.makePolygon(pts + [pts[0]])


def pocket(p, centre, r, z, height):
    """vertical prism: circle r round p plus a slot of width 2r running
    radially outward (away from centre) - room for the head and the hex key."""
    p = xy(p, z)
    radial = p - xy(centre, z)
    radial.normalize()
    side = V(-radial.y, radial.x, 0)
    cyl = Part.makeCylinder(r, abs(height), p if height > 0 else p + Z * height, Z)
    q0 = p - side * r
    L = 20.0
    box = Part.makePolygon([q0, q0 + radial * L, q0 + radial * L + side * 2 * r, q0 + side * 2 * r, q0])
    return cyl.fuse(Part.Face(box).extrude(Z * height))


def top_half_width():
    """largest A-aligned square (centred on A) that fits inside the column."""
    d = cB - cA
    bx, by = d.dot(LX), d.dot(LY)
    lim = R_COL - 0.5
    lo, hi = 0.0, R_COL
    for _ in range(60):
        w = (lo + hi) / 2
        far = max(math.hypot(sx * w - bx, sy * w - by) for sx in (-1, 1) for sy in (-1, 1))
        lo, hi = (w, hi) if far <= lim else (lo, w)
    return lo


def build() -> Part.Shape:
    t = math.tan(FLARE)
    z_a = ZA + T_A                      # top of the A plate
    z_b = ZB - T_B                      # underside of the B ring

    # A plate: rounded square round the four M4 holes
    outline_a = Part.Face(hull_wire([V(*b["position"]) for b in A_BOLTS], R_AFL, ZA))
    plate_a = outline_a.extrude(Z * T_A)

    # pyramid on the A plate (50 deg faces) running up into the column
    w_top = top_half_width()
    h_pyr = (HALF_A - w_top) * t
    pyr = Part.makeLoft([square_wire(HALF_A, z_a), square_wire(w_top, z_a + h_pyr)], True, True)
    pyr = pyr.common(outline_a.extrude(Z * (T_A + h_pyr + 1)))

    # column, and its 50 deg flare out to the B ring
    h_fl = (R_B_OUT - R_COL) * t
    z_fl = z_b - h_fl
    col = Part.makeCylinder(R_COL, z_fl - z_a, xy(cB, z_a), Z)
    flare = Part.makeCone(R_COL, R_B_OUT, h_fl, xy(cB, z_fl), Z)
    ring = Part.makeCylinder(R_B_OUT, T_B, xy(cB, z_b), Z)

    body = plate_a.fuse([pyr, col, flare, ring]).removeSplitter()

    # straight bore, open at both seats
    body = body.cut(Part.makeCylinder(R_HOLE, ZB - ZA + 2, xy(cB, ZA - 1), Z))

    # head pockets: bridged bearing faces, open outward for the key
    for b in A_BOLTS:
        body = body.cut(pocket(V(*b["position"]), cA, POCKET_A, z_a, 25.0))
    for b in B_BOLTS:
        body = body.cut(pocket(V(*b["position"]), cB, POCKET_B, z_b, -25.0))

    # clearance holes
    for b in A_BOLTS:
        body = body.cut(Part.makeCylinder(D_A / 2, T_A + 2, xy(V(*b["position"]), ZA - 1), Z))
    for b in B_BOLTS:
        body = body.cut(Part.makeCylinder(D_B / 2, T_B + 2, xy(V(*b["position"]), z_b - 1), Z))
    return body.removeSplitter()


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "valid", shape.isValid(),
          "solids", len(shape.Solids), "faces", len(shape.Faces))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
