"""Connecting part for flange-L3-s1, modelled in FreeCAD.

Built in the frame of face A: a = distance out of face A (along nA), b =
horizontal in face A, z = vertical.  Face B lies roughly at a = 63..73.

  * a solid square bridge (23 x 23) runs straight along a, from face A to
    face B, in the gap between bolts A4 and A5;
  * every bolt has its own boss (8 mm high, 12.4 mm dia) so the grip is 8 mm
    and the head seats on a flat top; nothing rises above a boss within the
    head and tool zones;
  * the bosses are joined to the bridge by 12 mm wide spokes at the boss depth.

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
BRIDGE_C = (11.0, -8.0)    # bridge axis in face-A frame (b, z)
BRIDGE_HALF = 11.5         # 23 x 23 solid bridge
GRIP = 8.0                 # boss depth = grip under the head (range 3.75..25 / 4.5..25)
BOSS_R = 6.2                # boss radius; spokes start at the boss centre (corners at r=6)
SPOKE_W = 12.0
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


def FA(a, b, z):
    """Point in the face-A frame: a along nA, b along tA, z vertical."""
    return OA + nA * a + tA * b + Z * z


def on_B_plane(b, z):
    """Point on face B at face-A coordinates (b, z)."""
    a = (OB - OA - tA * b - Z * z).dot(nB) / nA.dot(nB)
    return FA(a, b, z)


def bz_of(p):
    """Face-A (b, z) of a point."""
    v = p - OA
    return (v.dot(tA), v.z)


def prism(points, d):
    """Closed planar polygon of points, extruded along vector d."""
    wire = Part.makePolygon(points + [points[0]])
    return Part.Face(wire).extrude(d)


def quad_on_plane(c0, c1, width, normal):
    d = unit(c1 - c0)
    perp = unit(normal.cross(d)) * (width / 2.0)
    return [c0 + perp, c1 + perp, c1 - perp, c0 - perp]


def spoke(boss_c, target_c, normal, deep):
    """Spoke on a face from the boss centre to target_c, depth "deep"."""
    return prism(quad_on_plane(boss_c, target_c, SPOKE_W, normal), normal * deep)


def square(c, e1, e2, h):
    return [c + e1 * h + e2 * h, c - e1 * h + e2 * h,
            c - e1 * h - e2 * h, c + e1 * h - e2 * h]




def build():
    b0, z0 = BRIDGE_C
    h = BRIDGE_HALF
    # ---- bridge: solid square prism along nA ---------------------------------
    sq = [FA(0, b0 + h, z0 + h), FA(0, b0 - h, z0 + h),
          FA(0, b0 - h, z0 - h), FA(0, b0 + h, z0 - h)]
    parts = [prism(sq, nA * L_BRIDGE)]

    # ---- face A: bosses (on the bolt axis) and spokes to the bridge ---------
    centre_A = FA(0, b0, z0)
    for p in Ahole:
        parts.append(Part.makeCylinder(BOSS_R, GRIP, p, nA))
        parts.append(spoke(p, centre_A, nA, GRIP))

    # ---- face B: bosses and spokes to the bridge -----------------------------
    centre_B = on_B_plane(b0, z0)
    for p in Bhole:
        parts.append(Part.makeCylinder(BOSS_R, GRIP, p, nB))
        parts.append(spoke(p, centre_B, nB, GRIP))

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
