"""Connector for flange-L3-s1: a FreeCAD script (run: freecadcmd submission.py).

Concept
-------
Both mounting faces stand upright, so the part is designed to be printed along
the world Z axis, upside down: the bed is the plane z = z_bed, just above the
top bolt row of A.  Everything is then an upright prism with no overhang
steeper than 50 deg from the bed.

* A side: a 4 mm seat plate (bolt columns plus two ears for the side bolts,
  whose undersides rise at 50 deg).
* B side: a 4.8 mm seat plate with necks up to the two far bolts.  B's face
  ends 6 mm short of the bed, so the foot of the plate tapers away from the
  face plane and only touches the face from z_seat_top on.
* One torque-box beam joins them.  Its body lies in the height band that no
  bolt's tool path crosses, so it can be wide (stiff in sway and twist); it
  stands on the bed on a thin keel that threads the gap between the tool
  paths of A's and B's bolts, and widens into the body at 50 deg.
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

# ---- frames -----------------------------------------------------------------
Z = V(0, 0, 1)
OA = V(*FACES[0]["origin"]); NA = V(*FACES[0]["normal"])
OB = V(*FACES[1]["origin"]); NB = V(*FACES[1]["normal"])
SA = V(NA.y, -NA.x, 0)      # horizontal in-plane axis of A (bolt A2 is at s = +30)
UB = V(-NB.y, NB.x, 0)      # horizontal in-plane axis of B (bolt B4 is at u = +15)
ZA = OA.z                   # 5.5
ZB = OB.z                   # -18

# ---- parameters (mm) ----------------------------------------------------------
P = dict(
    tA=4.0,          # A plate thickness (M5 grip >= 3.75)
    tB=4.8,          # B plate thickness (M6 grip >= 4.5)
    dA=5.5, dB=6.6,  # clearance holes
    z_bed=35.9,      # print bed plane (part is printed upside down)
    z_top=-15.5,     # beam stops above the lower A bolts' tool path
    z_seat_top=28.8, # B seat starts here (face outline ends at 30, 1 mm inside = 29)
    foot=2.4,        # how far the B plate's foot stands off the face at the bed
    sC=2.0,          # beam centre on A
    uC=3.0,          # beam centre on B
    W=24.0,          # beam body width
    keel=2.4,        # keel thickness
    zf=23.0,         # keel starts widening here
    zB_top=-21.0,    # top of the B plate between the necks
)
P.update(json.loads(os.environ.get("PARAMS", "{}")))
T50 = math.tan(math.radians(50.0))


def a_pt(x, s, z):
    """world point from A frame: x along A's normal, s horizontal, z world."""
    return OA + NA * x + SA * s + Z * (z - ZA)


def b_pt(w, u, z):
    return OB + NB * w + UB * u + Z * (z - ZB)


def prism(pts, vec):
    wire = Part.makePolygon(list(pts) + [pts[0]])
    return Part.Face(wire).extrude(vec)


def a_plate():
    zb, t = P["z_bed"], T50
    zt = ZA - 30.4                                    # -24.9
    e = 34.4                                          # ear outer edge
    pts = [(-19.5, zb), (19.5, zb), (19.5, 12.0 + t * (e - 19.5)), (e, 12.0), (e, 1.1),
           (19.5, 1.1), (19.5, zt), (-19.5, zt), (-19.5, 1.1),
           (-e, 1.1), (-e, 9.9), (-19.5, 9.9 + t * (e - 19.5))]
    return prism([a_pt(0, s, z) for s, z in pts], NA * P["tA"])


def b_plate():
    zb, tB, zt = P["z_bed"], P["tB"], P["zB_top"]
    z0, zp = -63.4, -52.7          # far pads
    zn = zp + 2.5 * T50            # necks flare out to the pads at 50 deg
    pts = [(-20.5, zb), (20.5, zb), (20.5, zt), (18, zt), (18, zn), (20.5, zp), (20.5, z0),
           (9.5, z0), (9.5, zp), (12, zn), (12, zt), (-12, zt), (-12, zn), (-9.5, zp),
           (-9.5, z0), (-20.5, z0), (-20.5, zp), (-18, zn), (-18, zt), (-20.5, zt)]
    s = prism([b_pt(0, u, z) for u, z in pts], NB * tB)
    # tapered foot: clear of the face plane above the seat
    prof = [(-1, P["z_seat_top"]), (0, P["z_seat_top"]), (P["foot"], zb), (P["foot"], zb + 2), (-1, zb + 2)]
    return s.cut(prism([b_pt(w, -30, z) for w, z in prof], UB * 60))


def beam():
    zb, zt, zf = P["z_bed"], P["z_top"], P["zf"]
    pa = a_pt(0, P["sC"], 0); pb = b_pt(0, P["uC"], 0)
    d = pb - pa; d.normalize()
    q = V(-d.y, d.x, 0)
    hw, hk = P["W"] / 2, P["keel"] / 2
    zw = zf - T50 * (hw - hk)
    sec = [(-hw, zt), (hw, zt), (hw, zw), (hk, zf), (hk, zb), (-hk, zb), (-hk, zf), (-hw, zw)]
    L = (pb - pa).Length
    s = prism([pa - d * 20 + q * a + Z * z for a, z in sec], d * (L + 40))
    # trim the ends 2 mm inside the plates
    big = lambda f, x: prism([f(x, -90, -80), f(x, 90, -80), f(x - 60, 90, -80), f(x - 60, -90, -80)], Z * 160)
    return s.cut(big(a_pt, 2.0)).cut(big(b_pt, 2.5))


def build():
    s = a_plate().fuse(b_plate()).fuse(beam())
    for b in BOLTS:
        p = V(*b["position"]); ax = V(*b["axis"])
        d = P["dA"] if b["size"] == "M5" else P["dB"]
        s = s.cut(Part.makeCylinder(d / 2, 40, p - ax * 5, ax))
    return s.removeSplitter()


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    print("solids", len(shape.Solids), "valid", shape.isValid(), "volume", round(shape.Volume, 1),
          "faces", len(shape.Faces))
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
