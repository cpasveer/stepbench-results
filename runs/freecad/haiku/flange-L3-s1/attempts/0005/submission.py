"""Connecting part for flange-L3-s1, modelled in FreeCAD.

Built in the frame of face A: a = distance out of face A (along nA), b =
horizontal in face A, z = vertical.  Face B lies roughly at a = 63..73.

  * a solid square bridge (22 x 22) runs straight along a, from face A to
    face B, in the gap between bolts A4 and A5;
  * every bolt has its own boss (8 mm deep, 10 mm dia), grip 8 mm;
  * A bosses are joined to the bridge by 12 x 8 spokes lying on face A;
  * B bosses are joined to the bridge by steep struts (about 55 deg) so that no
    face hangs at a shallow angle when printed.

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
BRIDGE_R = 13.0            # solid round bridge (about the area of a 23 mm square)
GRIP = 8.0                 # boss depth = grip under the head (range 3.75..25 / 4.5..25)
BOSS_R = 6.2                # boss radius; spokes start at the boss centre (corners at r=6)
SPOKE_W = 11.0
R_CLEAR = 6.0              # spokes and struts leave the boss beyond this radius
STRUT_W = 10.0             # struts are 10 x 10 square sections
STRUT_SLOPE = 55.0         # degrees from horizontal (steeper than 45: printable)
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
    """Spoke on a face from outside the boss (R_CLEAR) to target_c."""
    return prism(quad_on_plane(boss_c, target_c, SPOKE_W, normal), normal * deep)


def square(c, e1, e2, h):
    return [c + e1 * h + e2 * h, c - e1 * h + e2 * h,
            c - e1 * h - e2 * h, c + e1 * h - e2 * h]


def strut(q, p, half):
    """Solid loft between two square sections centred on q and p (perpendicular to q->p)."""
    d = unit(p - q)
    e1 = unit(d.cross(Z if abs(d.z) < 0.9 else V(0, 1, 0)))
    e2 = unit(d.cross(e1))
    w1 = Part.makePolygon(square(q, e1, e2, half) + [square(q, e1, e2, half)[0]])
    w2 = Part.makePolygon(square(p, e1, e2, half) + [square(p, e1, e2, half)[0]])
    return Part.Solid(Part.makeLoft([w1, w2], True, True))


def build():
    b0, z0 = BRIDGE_C
    h = BRIDGE_R
    # ---- bridge: solid round bar along nA (spokes meet it radially) -----------
    parts = [Part.makeCylinder(BRIDGE_R, L_BRIDGE, FA(0, b0, z0), nA)]

    # ---- face A: bosses (on the bolt axis) and spokes to the bridge ---------
    centre_A = FA(0, b0, z0)
    for p in Ahole:
        parts.append(Part.makeCylinder(BOSS_R, GRIP, p, nA))
        parts.append(spoke(p, centre_A, nA, GRIP))

    # ---- face B: bosses and steep struts to the bridge ------------------------
    centre_B = on_B_plane(b0, z0)
    for p in Bhole:
        parts.append(Part.makeCylinder(BOSS_R, GRIP, p, nB))
        parts.append(spoke(p, centre_B, nB, GRIP))
    for p in []:
        pb, pz = bz_of(p)
        # direction from the boss towards the bridge in (b, z)
        dbz = (b0 - pb, z0 - pz)
        L = (dbz[0] ** 2 + dbz[1] ** 2) ** 0.5
        ub = (dbz[0] / L, dbz[1] / L)
        # boss-side end: 6 mm out from the bolt axis, 4 mm inside the boss
        pb_s, pz_s = pb + 6.0 * ub[0], pz + 6.0 * ub[1]
        P_end = on_B_plane(pb_s, pz_s) - nB * 4.0
        # bridge-side end: the bridge face nearest to the boss, inset 3 mm
        qb = min(max(pb, b0 - h + 3), b0 + h - 3)
        qz = min(max(pz, z0 - h + 3), z0 + h - 3)
        lat = ((pb_s - qb) ** 2 + (pz_s - qz) ** 2) ** 0.5
        dz_a = lat / __import__("math").tan(__import__("math").radians(STRUT_SLOPE))
        a_end = (P_end - OA).dot(nA)
        Q = FA(max(a_end - dz_a, 6.0), qb, qz)
        parts.append(strut(Q, P_end, STRUT_W / 2.0))

    # ---- face B tip of the bridge: the bridge reaches face B ------------------
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
