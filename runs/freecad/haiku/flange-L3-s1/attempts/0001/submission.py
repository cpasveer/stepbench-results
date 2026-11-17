"""Connecting part for flange-L3-s1, modelled in FreeCAD.

Structure (all dimensions mm):
  * face A side: a hub on the mounting face, six short bosses (one per M5
    clearance hole) and spokes joining them to the hub;
  * a hollow square-section bridge (1.2 mm walls) from the hub across the gap
    to face B, slanted toward B's bolt pattern;
  * face B side: a pad with an X of spokes to four bosses (one per M6 hole).
Every bolt has its own boss, so the head bears on a flat seat and the space
above the heads and the key swing zones stays clear.

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

# ---- design parameters -------------------------------------------------------
G_A = 5.0          # A-side thickness under the heads (grip, within 3.75..25)
G_B = 5.0          # B-side thickness under the heads (grip, within 4.5..25)
HUB_R = 8.0        # hub on face A
BOSS_R = 5.0       # boss around each bolt (seat ring is 2.75..4.25 for M5)
SPOKE_W = 4.5      # spoke width
BRIDGE_HALF = 7.0  # bridge outer half-size (14 x 14 section)
WALL = 1.2         # three 0.4 mm perimeters
B_PAD_R = 7.0
ROOT_BZ = (6.9, -4.0)   # bridge root on face A, in the gap between bolts A4 and A5
PAD_BZ = (25.0, -15.0)  # bridge landing on face B, between B's bolt rows
HOLE_DIA = {"M5": 5.5, "M6": 6.6}   # inside the clearance ranges


def unit(v):
    return v * (1.0 / v.Length)


def nv(t):
    return unit(V(t[0], t[1], t[2]))


nA = nv(FACES["A"]["normal"])
OA = V(*FACES["A"]["origin"])
nB = nv(FACES["B"]["normal"])
OB = V(*FACES["B"]["origin"])
uB = unit(Z.cross(nB))   # B's horizontal in-plane axis (B local x)
Ahole = [V(*b["position"]) for b in BOLTS if b["id"][0] == "A"]
Bhole = [V(*b["position"]) for b in BOLTS if b["id"][0] == "B"]


def FA(a, b, z):
    """A-face frame: a along nA (out of A), b along tA, z vertical."""
    tA = Z.cross(nA)
    return OA + nA * a + tA * b + Z * z


def FB(u, v, w):
    """B-face frame: u along uB, v vertical, w along nB (out of B)."""
    return OB + uB * u + Z * v + nB * w


def on_B_plane(b, z):
    """Point on face B at face-A coordinates (b, z): solves the B plane for a."""
    tA = Z.cross(nA)
    a = (OB - OA - tA * b - Z * z).dot(nB) / nA.dot(nB)
    return FA(a, b, z)


def prism(points, d):
    """Closed planar polygon of points, extruded along vector d."""
    wire = Part.makePolygon(points + [points[0]])
    return Part.Face(wire).extrude(d)


def spoke(c0, c1, width, normal, height):
    """Straight spoke in the face plane from c0 to c1, extruded along normal."""
    d = unit(c1 - c0)
    perp = unit(normal.cross(d)) * (width / 2.0)
    pts = [c0 + perp, c1 + perp, c1 - perp, c0 - perp]
    return prism(pts, normal * height)


def build():
    # ---- face A side ------------------------------------------------------
    parts = [Part.makeCylinder(HUB_R, G_A, FA(0, 0, 0), nA)]
    for p in Ahole:
        parts.append(Part.makeCylinder(BOSS_R, G_A, p, nA))
        parts.append(spoke(FA(0, 0, 0), p, SPOKE_W, nA, G_A))

    # ---- face B side ------------------------------------------------------
    q = on_B_plane(*PAD_BZ)
    parts.append(Part.makeCylinder(B_PAD_R, G_B, q, nB))
    for p in Bhole:
        parts.append(Part.makeCylinder(BOSS_R, G_B, p, nB))
        parts.append(spoke(q, p, SPOKE_W, nB, G_B))

    # ---- bridge: hollow square section from hub to pad ----------------------
    S = FA(2.5, *ROOT_BZ)
    E = q + nB * 2.5
    d = E - S
    L = d.Length
    dn = unit(d)
    e1 = unit(dn.cross(Z))
    e2 = unit(dn.cross(e1))

    def square(c, h):
        return [c + e1 * h + e2 * h, c - e1 * h + e2 * h,
                c - e1 * h - e2 * h, c + e1 * h - e2 * h]

    outer = prism(square(S, BRIDGE_HALF), d)
    cav0, cav1 = 6.0, L - 6.0
    inner = prism(square(S + dn * cav0, BRIDGE_HALF - WALL), dn * (cav1 - cav0))
    parts.append(outer.cut(inner))

    body = parts[0]
    for p in parts[1:]:
        body = body.fuse(p)

    # ---- bolt holes ---------------------------------------------------------
    for b in BOLTS:
        P = V(*b["position"])
        n = nv(b["axis"])
        r = HOLE_DIA[b["size"]] / 2.0
        body = body.cut(Part.makeCylinder(r, 40, P - n * 10, n))

    # ---- keep the part on the face side of each mounting plane -------------
    keep = Part.makeBox(300, 300, 300, V(-150, -150, -150))
    def half(plane_pt, n):
        # box whose face on plane_pt is the plane; material toward +n
        m = App.Matrix()
        # build a big slab: start at the plane, extend along +n
        slab = Part.makeBox(400, 400, 400, V(-200, -200, 0))
        rot = App.Rotation(V(0, 0, 1), n)
        slab.Placement = App.Placement(plane_pt, rot)
        return slab
    body = body.common(half(OA, nA)).common(half(OB, nB))

    body = body.Solids[0].removeSplitter() if len(body.Solids) == 1 else body
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "valid", shape.isValid())
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
