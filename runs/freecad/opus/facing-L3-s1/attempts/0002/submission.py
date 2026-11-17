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
R_B_OUT = 41.0       # B seat outer radius (face is 84.5 dia, keep 1 mm in)
R_B_IN = 27.0        # B seat inner radius
R_BASE = 33.0        # base cone radius on the A plate (centred on A)
H_BASE = 12.0        # base cone height
R_COL = 22.0         # column outer radius (centred on B)
R_HOLE = 19.0        # column bore
FLARE = math.radians(50)   # flare angle from horizontal (>45 for FDM)
POCKET_R = 5.0       # head pocket radius (head 8.5 + clearance)
D_A, D_B = 4.5, 5.5  # clearance holes


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


def build() -> Part.Shape:
    # A plate
    plate_a = Part.Face(hull_wire([V(*b["position"]) for b in A_BOLTS], R_AFL, ZA)).extrude(V(0, 0, T_A))

    # base cone: circle on A's centre -> column circle on B's centre
    z0 = ZA + T_A
    z1 = z0 + H_BASE
    w0 = Part.Wire(Part.makeCircle(R_BASE, xy(cA, z0), Z))
    w1 = Part.Wire(Part.makeCircle(R_COL, xy(cB, z1), Z))
    base = Part.makeLoft([w0, w1], True, True)

    # column and flare
    z_flare_top = ZB - T_B
    h_flare = (R_B_OUT - R_COL) * math.tan(FLARE)
    z_flare = z_flare_top - h_flare
    col = Part.makeCylinder(R_COL, z_flare - z1 + 0.0, xy(cB, z1), Z)
    flare = Part.makeCone(R_COL, R_B_OUT, h_flare, xy(cB, z_flare), Z)
    ring = Part.makeCylinder(R_B_OUT, T_B, xy(cB, z_flare_top), Z)

    body = plate_a.fuse([base, col, flare, ring]).removeSplitter()

    # bore: straight, then opening upward at the flare angle to the seat ring
    h_open = (R_B_IN - R_HOLE) * math.tan(FLARE)
    z_open = ZB - h_open
    bore = Part.makeCylinder(R_HOLE, z_open - (ZA - 1), xy(cB, ZA - 1), Z)
    bore = bore.fuse(Part.makeCone(R_HOLE, R_B_IN + 1 / math.tan(FLARE), h_open + 1, xy(cB, z_open), Z))
    body = body.cut(bore)

    # head pockets under B: round end at the bolt, open radially outward
    depth = 40.0
    for b in B_BOLTS:
        p = xy(V(*b["position"]), z_flare_top)
        radial = p - xy(cB, z_flare_top)
        radial.normalize()
        side = V(-radial.y, radial.x, 0)
        cyl = Part.makeCylinder(POCKET_R, depth, p - Z * depth, Z)
        L = 15.0
        q0 = p - side * POCKET_R
        box = Part.makePolygon([q0, q0 + radial * L, q0 + radial * L + side * 2 * POCKET_R,
                                q0 + side * 2 * POCKET_R, q0])
        slot = Part.Face(box).extrude(V(0, 0, -depth))
        body = body.cut(cyl.fuse(slot))

    # clearance holes
    for b in A_BOLTS:
        body = body.cut(Part.makeCylinder(D_A / 2, T_A + 2, xy(V(*b["position"]), ZA - 1), Z))
    for b in B_BOLTS:
        body = body.cut(Part.makeCylinder(D_B / 2, T_B + 2, xy(V(*b["position"]), ZB - T_B - 1), Z))
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
