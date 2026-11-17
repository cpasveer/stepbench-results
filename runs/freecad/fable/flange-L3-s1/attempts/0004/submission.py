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
    tA=4.0,          # A plate thickness (M5 grip 3.75-25)
    tB=8.0,          # B plate thickness (M6 grip 4.5-25); B's bolts are tied together by it
    dA=5.5, dB=6.6,  # clearance holes
    z_bed=35.9,      # print bed plane (part is printed upside down)
    z_top=-15.5,     # beam top: just clear of the lower A bolts' tool path
    z_seat_top=28.8, # B seat starts here (face outline ends at 30, 1 mm inside = 29)
    foot=2.4,        # how far the B plate's foot stands off the face at the bed
    uK=3.0,          # keel position on B (the beam runs square to B)
    uL=-15.3,        # beam body sides on B
    uR=9.4,          #   (uR is flush with the right fin and the right pad column)
    hmin=8.0,        # height of the beam's upright side at uL
    keel=1.6,        # keel thickness
    fin_u=8.2,       # buttress fins to the far B bolts, at +-fin_u
    fin_t=2.4,
    finL=16.0, finR=16.0, fin_end=4.0,   # fin depth at the beam / at the far end
    neck=13.5,       # width of B's legs to the far bolts (13.5 = full width; the fins stand on them)
    slots=0,         # open the B plate beside the keel
)
P.update(json.loads(os.environ.get("PARAMS", "{}")))
T50 = math.tan(math.radians(50.0))
P["zf"] = P["z_top"] + P["hmin"] + T50 * (P["uK"] - P["keel"] / 2 - P["uL"])   # keel starts widening here


def a_pt(x, s, z):
    """world point from A frame: x along A's normal, s horizontal, z world."""
    return OA + NA * x + SA * s + Z * (z - ZA)


def b_pt(w, u, z):
    return OB + NB * w + UB * u + Z * (z - ZB)


def prism(pts, vec):
    wire = Part.makePolygon(list(pts) + [pts[0]])
    return Part.Face(wire).extrude(vec)


def flank(u):
    """lower (print-bottom) boundary of the beam section at u."""
    hk = P["keel"] / 2
    return P["zf"] - T50 * max(0.0, abs(u - P["uK"]) - hk)


def a_plate():
    zb, t, zt = P["z_bed"], T50, ZA - 30.4
    e = 34.4                                          # ear outer edge
    zs = 9.9 + t * (e - 19.5)
    zq = P["z_top"]

    def foot(u):                                      # where the beam side u lands on A
        p = b_pt(0, u, 0)
        return (p - NB * ((p - OA).dot(NA) - P["tA"]) / NB.dot(NA) - OA).dot(SA)

    # the plate stops 5 mm or more outside the beam; beside a wide beam it grows a wing, flush with the beam top
    wr = max(foot(P["uR"]) + 5.1, 24.7) if foot(P["uR"]) > 14.4 else None
    wl = min(foot(P["uL"]) - 5.1, -24.7) if foot(P["uL"]) < -14.4 else None
    right = [(19.5, zs), (e, 9.9), (e, 1.1)] + ([(wr, 1.1), (wr, zq), (19.5, zq)] if wr else [(19.5, 1.1)])
    left = ([(-19.5, zq), (wl, zq), (wl, 1.1)] if wl else [(-19.5, 1.1)]) + [(-e, 1.1), (-e, 9.9), (-19.5, zs)]
    pts = [(-19.5, zb), (19.5, zb)] + right + [(19.5, zt), (-19.5, zt)] + left
    return prism([a_pt(0, s, z) for s, z in pts], NA * P["tA"])


def b_plate():
    zb, tB, zq, hk = P["z_bed"], P["tB"], P["z_top"], P["keel"] / 2
    z0, zp = -63.4, -52.7
    uk, ur = P["uK"], min(P["uR"], 9.5)        # right pad column: flush with a narrow beam
    ui = P["fin_u"] - P["fin_t"] / 2               # necks' inner edge = fins' inner face
    un = ui + P["neck"]
    zn = zp + T50 * (20.5 - un)
    bed = [(-20.5, zb), (20.5, zb)]
    if P["slots"]:   # open slots beside the keel, under the flanks
        bed = [(-20.5, zb), (-9.5, zb), (-9.5, flank(-9.5)), (uk - hk, P["zf"]), (uk - hk, zb),
               (uk + hk, zb), (uk + hk, P["zf"]), (ur, flank(ur)), (ur, zb), (20.5, zb)]
    if un < 20.4:    # necks, flaring out to the far pads at 50 deg
        rleg = [(20.5, zq), (un, zq), (un, zn), (20.5, zp), (20.5, z0), (ui, z0), (ui, zq)]
        lleg = [(-ui, zq), (-ui, z0), (-20.5, z0), (-20.5, zp), (-un, zn), (-un, zq), (-20.5, zq)]
    else:            # full-width legs
        rleg = [(20.5, z0), (ui, z0), (ui, zq)]
        lleg = [(-ui, zq), (-ui, z0), (-20.5, z0)]
    pts = bed + rleg + lleg
    s = prism([b_pt(0, u, z) for u, z in pts], NB * tB)
    # tapered foot: clear of the face plane above the seat
    prof = [(-1, P["z_seat_top"]), (0, P["z_seat_top"]), (P["foot"], zb), (P["foot"], zb + 2), (-1, zb + 2)]
    return s.cut(prism([b_pt(w, -30, z) for w, z in prof], UB * 60))


def beam():
    """torque box square to B: flat top, 50 deg flanks down to a keel that stands on the bed."""
    zb, zq, zf, hk = P["z_bed"], P["z_top"], P["zf"], P["keel"] / 2
    uk, ul, ur = P["uK"], P["uL"], P["uR"]
    sec = [(ul, zq), (ur, zq), (ur, flank(ur)), (uk + hk, zf), (uk + hk, zb),
           (uk - hk, zb), (uk - hk, zf), (ul, flank(ul))]
    s = prism([b_pt(P["tB"] - 2.0, u, z) for u, z in sec], NB * 90)
    big = prism([a_pt(P["tA"] - 2.0, -90, -80), a_pt(P["tA"] - 2.0, 90, -80), a_pt(-60, 90, -80), a_pt(-60, -90, -80)], Z * 160)
    return s.cut(big)


def fin(side):
    zq, tB, z0 = P["z_top"], P["tB"], -63.4
    d = P["finL"] if side < 0 else P["finR"]
    u0 = side * P["fin_u"] - P["fin_t"] / 2
    prof = [(tB - 2.0, zq), (tB + d, zq), (tB + P["fin_end"], z0), (tB - 2.0, z0)]
    return prism([b_pt(w, u0, z) for w, z in prof], UB * P["fin_t"])


def build():
    s = a_plate().fuse(b_plate()).fuse(beam()).fuse(fin(-1)).fuse(fin(+1))
    for b in BOLTS:
        p = V(*b["position"]); ax = V(*b["axis"])
        d = P["dA"] if b["size"] == "M5" else P["dB"]
        s = s.cut(Part.makeCylinder(d / 2, 40, p - ax * 5, ax))
    return s.removeSplitter()


OUT = os.environ.get("OUT_STEP") or os.path.join(HERE, "output.step")
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
