"""Connecting part for flange-L3-s1, modelled in FreeCAD.

Built in the frame of face A: a = distance out of face A (along nA), b =
horizontal in face A, z = vertical.  Face B lies roughly at a = 70.

  * a solid square bridge (20 x 20) runs straight along a, from face A to
    face B, in the gap between bolts A4 and A5;
  * every bolt has its own boss (5 mm high, 10 mm dia) on the face, joined to
    the bridge by a spoke (12 mm wide; 5 mm deep near the boss so the head
    stays clear, 8 mm deep further out for stiffness).

Run:  freecadcmd submission.py   (writes output.step next to this file)
"""
import json
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
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
V = App.Vector
Z = V(0, 0, 1)

# ---- design parameters (mm) ---------------------------------------------------
BRIDGE_C = (12.0, -10.0)   # bridge axis on the face-A frame (b, z)
BRIDGE_HALF = 10.0         # 20 x 20 solid bridge
BOSS_R = 5.0               # boss (seat ring 2.75..4.25 for M5, 3.25..5 for M6)
G_LOW = 5.0                # boss height = grip under the head
SPOKE_W = 12.0
SPOKE_DEEP = 8.0           # spoke depth beyond the head clearance zone
R_CLEAR = 6.0              # spoke leaves the boss beyond this radius
HOLE_DIA = {"M5": 5.5, "M6": 6.6}   # inside the clearance ranges
L_BRIDGE = 80.0            # long enough to reach face B; trimmed to it


def unit(v):
    return v * (1.0 / v.Length)


def nv(t):
    return unit(V(t[0], t[1], t[2]))


nA = nv(FACES["A"]["normal"])
OA = V(*FACES["A"]["origin"])
nB = nv(FACES["B"]["normal"])
OB = V(*FACES["B"]["origin"])
tA = Z.cross(nA)
Ahole = [V(*b["position"]) for b in BOLTS if b["id"][0] == "A"]
Bhole = [V(*b["position"]) for b in BOLTS if b["id"][0] == "B"]
Abolt = [b for b in BOLTS if b["id"][0] == "A"]
Bbolt = [b for b in BOLTS if b["id"][0] == "B"]


def FA(a, b, z):
    """Point in the face-A frame: a along nA, b along tA, z vertical."""
    return OA + nA * a + tA * b + Z * z


def on_B_plane(b, z):
    """Point on face B at face-A coordinates (b, z)."""
    a = (OB - OA - tA * b - Z * z).dot(nB) / nA.dot(nB)
    return FA(a, b, z)


def prism(points, d):
    """Closed planar polygon of points, extruded along vector d."""
    wire = Part.makePolygon(points + [points[0]])
    return Part.Face(wire).extrude(d)


def quad_on_plane(c0, c1, width, normal):
    d = unit(c1 - c0)
    perp = unit(normal.cross(d)) * (width / 2.0)
    return [c0 + perp, c1 + perp, c1 - perp, c0 - perp]


def spoke(boss_c, target_c, normal, deep, plane_pt_bridge=None):
    """Spoke from outside a boss (radius R_CLEAR) to target_c, depth 'deep'."""
    d = unit(target_c - boss_c)
    start = boss_c + d * R_CLEAR
    if (target_c - start).Length < 1.0:
        return None
    return prism(quad_on_plane(start, target_c, SPOKE_W, normal), normal * deep)


def low_tab(boss_c, target_c, normal):
    """Low (grip-height) connection from the boss centre out past the head zone."""
    d = unit(target_c - boss_c)
    end = boss_c + d * (R_CLEAR + 0.5)
    return prism(quad_on_plane(boss_c, end, SPOKE_W, normal), normal * G_LOW)


def build():
    # ---- bridge: solid square prism along nA, from face A to face B ---------
    b0, z0 = BRIDGE_C
    h = BRIDGE_HALF
    sq = [FA(0, b0 + h, z0 + h), FA(0, b0 - h, z0 + h),
          FA(0, b0 - h, z0 - h), FA(0, b0 + h, z0 - h)]
    parts = [prism(sq, nA * L_BRIDGE)]

    # ---- face A side: bosses and spokes -------------------------------------
    centre_A = FA(0, b0, z0)
    for p in Ahole:
        parts.append(Part.makeCylinder(BOSS_R, G_LOW, p, nA))
        parts.append(low_tab(p, centre_A, nA))
        s = spoke(p, centre_A, nA, SPOKE_DEEP)
        if s is not None:
            parts.append(s)

    # ---- face B side: bosses and spokes -------------------------------------
    centre_B = on_B_plane(b0, z0)
    for p in Bhole:
        parts.append(Part.makeCylinder(BOSS_R, G_LOW, p, nB))
        parts.append(low_tab(p, centre_B, nB))
        s = spoke(p, centre_B, nB, SPOKE_DEEP)
        if s is not None:
            parts.append(s)

    body = parts[0]
    for p in parts[1:]:
        body = body.fuse(p)

    # ---- bolt clearance holes ----------------------------------------------
    for b in BOLTS:
        P = V(*b["position"])
        n = nv(b["axis"])
        r = HOLE_DIA[b["size"]] / 2.0
        body = body.cut(Part.makeCylinder(r, 40, P - n * 10, n))

    # ---- keep the part on the face side of each mounting plane -------------
    def half(origin, n):
        slab = Part.makeBox(400, 400, 400, V(-200, -200, 0))
        slab.Placement = App.Placement(origin, App.Rotation(Z, n))
        return slab
    body = body.common(half(OA, nA)).common(half(OB, nB))
    if len(body.Solids) == 1:
        body = body.Solids[0].removeSplitter()
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    shape.exportStep(OUT)
    print("wrote", OUT, "solids", len(shape.Solids), "volume", round(shape.Volume, 1), "valid", shape.isValid())
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
