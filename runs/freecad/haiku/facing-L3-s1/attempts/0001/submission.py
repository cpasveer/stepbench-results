"""Connecting part between fixed A (square, 4x M4 below) and fixed B (round, 3x M5 above).

Design: a solid central hub spans the 68.5 mm gap from A's face to B's face.
Two flat "stars" carry the bolts: a 6 mm pad-and-arm star lies on A's face
(bolts M4 come up through it), and a 6 mm star hangs under B's face (bolts
M5 come down through it). Pads sit under the bolt heads' footprints only;
heads are kept free of material by construction.
Run: freecadcmd submission.py  -> writes output.step next to this file.
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

# ---- design parameters (mm) ----
Z_A = -94.0          # A mounting face (A body below)
Z_B = -25.5          # B mounting face (B body above)
T_A = 6.0            # bottom star thickness (grip under M4 heads)
T_B = 6.0            # top star thickness (grip under M5 heads)
HUB_R = 8.0          # central solid hub radius
PAD_RA = 5.0         # pad radius around M4 bolts (keeps seat 1 mm inside A face)
PAD_RB = 6.0         # pad radius around M5 bolts
ARM_WA = 6.0         # arm width, bottom star
ARM_WB = 8.0         # arm width, top star
HOLE_D = {"M4": 4.5, "M5": 5.5}


def bolt_xy(b):
    return (b["position"][0], b["position"][1])


def arm(p0, p1, w, z0, t):
    """Flat bar of width w from p0 to p1 (xy), extruded from z0 up by t."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L * w / 2, dx / L * w / 2
    pts = [V(p0[0] + nx, p0[1] + ny, z0), V(p1[0] + nx, p1[1] + ny, z0),
           V(p1[0] - nx, p1[1] - ny, z0), V(p0[0] - nx, p0[1] - ny, z0)]
    pts.append(pts[0])
    face = Part.Face(Part.makePolygon(pts))
    return face.extrude(V(0, 0, t))


def build() -> Part.Shape:
    fa = [f for f in FACES if f["part"] == "A"][0]
    fb = [f for f in FACES if f["part"] == "B"][0]
    assert abs(fa["origin"][2] - Z_A) < 1e-6 and abs(fb["origin"][2] - Z_B) < 1e-6
    bolts_a = [b for b in BOLTS if b["id"].startswith("A")]
    bolts_b = [b for b in BOLTS if b["id"].startswith("B")]

    # hub centre: midway between the two bolt patterns' centres
    ca = fa["origin"][:2]
    cb = fb["origin"][:2]
    P = ((ca[0] + cb[0]) / 2, (ca[1] + cb[1]) / 2)

    parts = [Part.makeCylinder(HUB_R, Z_B - Z_A, V(P[0], P[1], Z_A), V(0, 0, 1))]
    # bottom star on A
    for b in bolts_a:
        q = bolt_xy(b)
        parts.append(Part.makeCylinder(PAD_RA, T_A, V(q[0], q[1], Z_A), V(0, 0, 1)))
        parts.append(arm(P, q, ARM_WA, Z_A, T_A))
    # top star on B
    for b in bolts_b:
        q = bolt_xy(b)
        parts.append(Part.makeCylinder(PAD_RB, T_B, V(q[0], q[1], Z_B - T_B), V(0, 0, 1)))
        parts.append(arm(P, q, ARM_WB, Z_B - T_B, T_B))

    body = parts[0]
    for p in parts[1:]:
        body = body.fuse(p)
    body = body.removeSplitter()

    # clearance holes along every bolt axis, through the pads
    for b in BOLTS:
        q = bolt_xy(b)
        d = HOLE_D[b["size"]]
        ax = b["axis"]
        if ax[2] > 0:   # A bolts: hole from A's face upward
            base = V(q[0], q[1], Z_A - 1)
        else:           # B bolts: hole from B's face downward
            base = V(q[0], q[1], Z_B + 1)
        body = body.cut(Part.makeCylinder(d / 2, abs(Z_B - Z_A) + 2, base, V(ax[0], ax[1], ax[2])))
    return body.removeSplitter()


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
