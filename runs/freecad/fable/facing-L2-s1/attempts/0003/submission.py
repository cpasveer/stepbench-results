"""facing-L2-s1: box-section connector between a square mount (A, below) and a
round mount (B, above).

Concept: two flange plates joined by two parallel webs -> a closed box section.
Everything is a prism along one horizontal direction (A's local +Y), so the
part prints standing on the flat end of that prism: plates and webs are all
vertical walls, no support.  The round flange gets a flat foot in the bed plane with two
short 50-degree flanks, so its rim rises from the bed self-supporting.
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
SETBACK_A = 2.0    # square flange edge inside A's face outline
SETBACK_B = 1.5    # round flange edge inside B's face outline
D_M4 = 4.5
D_M5 = 5.5
T_WEB = 5.0
WEB_X = (-18.0, 10.0)   # web mid-planes, A-local x (clear of every bolt head)
WEB_END = -27.0    # webs stop here (A-local y), inside the round flange
R_FIL = 4.0        # web-to-flange fillets
FLANK = 50.0       # deg from the bed: flanks of the round flange's foot
FLANK_DIST = 39.25 # flank distance from B's centre (M5 head seat reaches 39.12)


def local_frame():
    """A's frame: world = o + R @ local.  Returns (o, R rows)."""
    return PL["A"]["origin"], PL["A"]["rotation"]


def to_local(p, o, R):
    d = [p[i] - o[i] for i in range(3)]
    return [sum(R[r][c] * d[r] for r in range(3)) for c in range(3)]


def build() -> Part.Shape:
    o, R = local_frame()
    half = TASK["fixed_parts"][0]["interface"]["face_size"][0] / 2 - SETBACK_A
    rb = TASK["fixed_parts"][1]["interface"]["face_size"][0] / 2 - SETBACK_B
    bc = to_local(PL["B"]["origin"], o, R)
    H = bc[2]                       # gap between the two mounting faces
    bed = half                      # print bed plane: local y = bed, build -y

    # flange on A: square, all round inside the mounting face
    plate_a = Part.makeBox(2 * half, 2 * half, T_A, V(-half, -half, 0))

    # flange on B: disc with a flat foot in the bed plane and two short
    # flanks at FLANK degrees from the bed, so the rim rises self-supporting.
    # The flanks pass just outside the nearest bolt head's seat.
    disc = Part.makeCylinder(rb, T_B, V(bc[0], bc[1], H - T_B))
    a = math.radians(FLANK)
    xf = (FLANK_DIST - (bed - bc[1]) * math.cos(a)) / math.sin(a)  # foot half width
    big = 3 * rb
    pts = [V(bc[0] - xf, bed, 0), V(bc[0] + xf, bed, 0),
           V(bc[0] + xf + big * math.cos(a), bed - big * math.sin(a), 0),
           V(bc[0] - xf - big * math.cos(a), bed - big * math.sin(a), 0)]
    keep = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, 0, H))
    plate_b = disc.common(keep)

    # two webs from the bed up, stopped where the round flange runs out
    wl = bed - WEB_END
    body = plate_a.fuse(plate_b)
    for x in WEB_X:
        body = body.fuse(Part.makeBox(T_WEB, wl, H, V(x - T_WEB / 2, WEB_END, 0)))

    # fillets where the webs meet the flanges: concave quarter-round fills,
    # prisms along the print axis like everything else
    for x in WEB_X:
        for sx in (-1, 1):
            for zc, sz in ((T_A, 1), (H - T_B, -1)):
                xc = x + sx * T_WEB / 2
                blk = Part.makeBox(R_FIL, wl, R_FIL,
                                   V(min(xc, xc + sx * R_FIL), WEB_END,
                                     min(zc, zc + sz * R_FIL)))
                rod = Part.makeCylinder(R_FIL, wl + 2,
                                        V(xc + sx * R_FIL, WEB_END - 1, zc + sz * R_FIL),
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

    # A's frame is the world frame turned about the vertical: place rigidly
    ang = math.degrees(math.atan2(R[1][0], R[0][0]))
    body.Placement = App.Placement(V(*o), App.Rotation(V(0, 0, 1), ang))
    return body


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
