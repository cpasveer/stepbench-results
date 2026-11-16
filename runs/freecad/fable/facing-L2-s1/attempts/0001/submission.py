"""facing-L2-s1: box-section connector between a square mount (A, below) and a
round mount (B, above).

Concept: two flange plates joined by two parallel webs -> a closed box section.
Everything is a prism along one horizontal direction (A's local +Y), so the
part prints standing on the flat end of that prism: plates and webs are all
vertical walls, no support.  The round flange is trimmed to a flat foot with
50-degree flanks so it rises from the bed without a shallow overhang.
The webs sit between the bolt rows, leaving every head free for its hex key.

    freecadcmd submission.py     # writes output.step
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
POSE0 = TASK["poses"][0]
PL = POSE0["placements"]
BOLTS = POSE0["resolved"]["bolts"]
V = App.Vector

# ---- parameters (mm) -------------------------------------------------------
T_A = 6.0          # flange on A (M4 grip 3..25)
T_B = 6.0          # flange on B (M5 grip 3.75..25)
SETBACK = 2.0      # flange edge inside the mounting face outline
D_M4 = 4.5
D_M5 = 5.5
T_WEB = 5.0
WEB_X = (-18.0, 10.0)   # web mid-planes, A-local x (clear of every bolt head)
R_FIL = 4.0        # web-to-flange fillets
FLANK = 50.0       # deg from the bed: flanks of the round flange's foot
FOOT = 8.0         # half width of the round flange's flat foot


def local_frame():
    """A's frame: world = o + R @ local.  Returns (o, R rows)."""
    return PL["A"]["origin"], PL["A"]["rotation"]


def to_local(p, o, R):
    d = [p[i] - o[i] for i in range(3)]
    return [sum(R[r][c] * d[r] for r in range(3)) for c in range(3)]


def build() -> Part.Shape:
    o, R = local_frame()
    half = TASK["fixed_parts"][0]["interface"]["face_size"][0] / 2 - SETBACK
    rb = TASK["fixed_parts"][1]["interface"]["face_size"][0] / 2 - SETBACK + 1.0
    bc = to_local(PL["B"]["origin"], o, R)
    H = bc[2]                       # gap between the two mounting faces
    bed = -half                     # print bed plane: local y = bed

    # flange on A: square, all round inside the mounting face
    plate_a = Part.makeBox(2 * half, 2 * half, T_A, V(-half, -half, 0))

    # flange on B: disc, trimmed to a flat foot on the bed with steep flanks
    disc = Part.makeCylinder(rb, T_B, V(bc[0], bc[1], H - T_B))
    k = math.tan(math.radians(FLANK))
    big = 3 * rb
    pts = [V(bc[0] - FOOT, bed, 0), V(bc[0] + FOOT, bed, 0),
           V(bc[0] + FOOT + big, bed + big * k, 0),
           V(bc[0] - FOOT - big, bed + big * k, 0)]
    keep = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, 0, H))
    plate_b = disc.common(keep)

    # two webs, full length of the square flange, bed to top
    body = plate_a.fuse(plate_b)
    for x in WEB_X:
        body = body.fuse(Part.makeBox(T_WEB, 2 * half, H,
                                      V(x - T_WEB / 2, -half, 0)))
    # fillets where the webs meet the flanges: concave quarter-round fills,
    # prisms along the print axis like everything else
    for x in WEB_X:
        for sx in (-1, 1):
            for zc, sz in ((T_A, 1), (H - T_B, -1)):
                xc = x + sx * T_WEB / 2
                blk = Part.makeBox(R_FIL, 2 * half, R_FIL,
                                   V(min(xc, xc + sx * R_FIL), -half,
                                     min(zc, zc + sz * R_FIL)))
                rod = Part.makeCylinder(R_FIL, 2 * half + 2,
                                        V(xc + sx * R_FIL, -half - 1, zc + sz * R_FIL),
                                        V(0, 1, 0))
                body = body.fuse(blk.cut(rod))
    body = body.removeSplitter()

    # clearance holes
    for b in BOLTS:
        p = to_local(b["position"], o, R)
        d = D_M4 if b["size"] == "M4" else D_M5
        z0 = -1.0 if p[2] < H / 2 else H - T_B - 1.0
        t = (T_A if p[2] < H / 2 else T_B) + 2.0
        body = body.cut(Part.makeCylinder(d / 2, t, V(p[0], p[1], z0)))

    m = App.Matrix(R[0][0], R[0][1], R[0][2], o[0],
                   R[1][0], R[1][1], R[1][2], o[1],
                   R[2][0], R[2][1], R[2][2], o[2],
                   0, 0, 0, 1)
    return body.transformed(m)


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    assert len(shape.Solids) == 1 and shape.isValid()
    shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "faces", len(shape.Faces))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
