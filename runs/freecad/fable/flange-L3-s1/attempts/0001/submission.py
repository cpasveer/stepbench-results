"""Connector for flange-L3-s1: a FreeCAD script (run: freecadcmd submission.py).

Concept
-------
Both mounting faces stand upright, so the part is designed to be printed along
the world Z axis, upside down (the bed is the plane z = Z_BED, just above the
top bolt row of A).  Everything is then a vertical wall:

* A side: a 4 mm seat plate reduced to two bolt columns, two ears for the side
  bolts (their undersides rise at 50 deg) and a short tie between the columns.
* B side: two bolt strips (pad - neck - pad).  They stand on the bed, but B's
  face ends 6 mm short of it, so the strips are relieved 1 mm off the face
  there and come onto the face over a 50 deg chamfer.
* Two upright webs carry the load.  The left web runs square to B between its
  bolt columns; the right web runs from the land between A's bolt columns to
  the outer edge of B, giving a wide stance against twisting.  Where a web
  crosses the tool path of a bolt it has a gabled doorway (50 deg roof), and
  both webs stop short of the lower A bolts.
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
    tw=2.4,          # web thickness
    z_bed=35.9,      # print bed plane (part is printed upside down)
    z_web_top=-15.5, # webs stop above the lower A bolts' tool path
    relief=1.0,      # B strips stand off the face above z_face_top
    z_seat_top=28.8, # B seat starts here (face outline ends at 30, 1 mm inside = 29)
    uL=-8.0,         # left web position on B
    uR=23.5,         # right web position on B
    sR=23.8,         # right web position on A
    zR_A=22.5,       # right web lower (print-bottom) edge at the A plate
    door_z=26.7,     # doorway springing height (tool path of A1/A6 ends at 27.23)
    door_x0=12.5, door_x1=40.0,
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
           (25.9, 1.1), (25.9, -16.5), (19.5, -16.5), (19.5, zt), (10.5, zt),
           (10.5, zb - 10.0), (-3.5, zb - 10.0), (-3.5, zt), (-19.5, zt), (-19.5, 1.1),
           (-e, 1.1), (-e, 9.9), (-19.5, 9.9 + t * (e - 19.5))]
    return prism([a_pt(0, s, z) for s, z in pts], NA * P["tA"])


def b_strip(side):
    zb, t, tB = P["z_bed"], T50, P["tB"]
    z0, z1, zp = -63.4, 16.5, -52.7
    if side < 0:
        uo, ui = -20.5, P["uL"] + P["tw"] / 2 + 1.0   # outer / inner edge
        un = ui - 6.0                                   # neck edge
        pts = [(uo, zb), (ui, zb), (ui, z0), (uo, z0), (uo, zp),
               (un, zp + t * (un - uo)), (un, z1), (uo, z1)]
    else:
        ui, uo = 9.5, P["uR"] + P["tw"] / 2 + 1.0
        un = uo - 6.0
        pts = [(ui, zb), (uo, zb), (uo, z0), (ui, z0), (ui, zp),
               (un, zp + t * (un - ui)), (un, z1), (ui, z1)]
    s = prism([b_pt(0, u, z) for u, z in pts], NB * tB)
    # relief off the face above the seat, with a printable chamfer
    r, zs = P["relief"], P["z_seat_top"]
    prof = [(-1, zs), (0, zs), (r, zs + r * t), (r, zb + 2), (-1, zb + 2)]
    s = s.cut(prism([b_pt(w, -30, z) for w, z in prof], UB * 60))
    if side > 0:  # the outer margin of the right strip stays off the face
        s = s.cut(prism([b_pt(-1, 21.0, z0 - 2), b_pt(r, 21.0, z0 - 2),
                         b_pt(r, 21.0, zb + 2), b_pt(-1, 21.0, zb + 2)], UB * 10))
    return s


def web(pa, pb):
    """upright wall with centreline pa -> pb (plan points), ends buried in the plates."""
    d = (pb - pa); d.z = 0; d.normalize()
    nrm = V(-d.y, d.x, 0)
    h = P["tw"] / 2
    pts = []
    for off, order in ((h, 0), (-h, 1)):
        q = pa + nrm * off
        ta = (2.0 - (q - OA).dot(NA)) / d.dot(NA)          # plane 2 mm inside A plate
        tb = (2.5 - (q - OB).dot(NB)) / d.dot(NB)          # plane 2.5 mm inside B strip
        e = [q + d * ta, q + d * tb]
        pts += e if order == 0 else e[::-1]
    pts = [V(p.x, p.y, P["z_web_top"]) for p in pts]
    return prism(pts, Z * (P["z_bed"] - P["z_web_top"]))


def doorway(prof):
    return prism([a_pt(x, -70, z) for x, z in prof], SA * 140)


def build():
    zb, t = P["z_bed"], T50
    parts = [a_plate(), b_strip(-1), b_strip(+1)]

    # left web: square to B at u = uL
    wl = web(b_pt(60, P["uL"], 0), b_pt(0, P["uL"], 0))
    x0, x1, dz = P["door_x0"], P["door_x1"], P["door_z"]
    xm = (x0 + x1) / 2
    wl = wl.cut(doorway([(x0, zb + 2), (x0, dz), (xm, dz - t * (xm - x0)), (x1, dz), (x1, zb + 2)]))
    parts.append(wl)

    # right web: from the land between A's bolt columns to the outer edge of B
    wr = web(a_pt(P["tA"], P["sR"], 0), b_pt(0, P["uR"], 0))
    za = P["zR_A"]; xa = P["tA"]
    xm = (za + t * xa - dz + t * x1) / (2 * t)
    zm = za - t * (xm - xa)
    wr = wr.cut(doorway([(1.0, zb + 2), (1.0, za + t * (xa - 1.0)), (xm, zm), (x1, dz), (x1, zb + 2)]))
    parts.append(wr)

    s = parts[0]
    for p in parts[1:]:
        s = s.fuse(p)
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
