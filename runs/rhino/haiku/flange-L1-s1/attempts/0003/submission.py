#! python3
"""Connecting part between the round flange A and the rectangular block B.

Design: two seat plates (one per mounting face) joined by a single strut.
  - plate A: 72 mm round pad, 11 mm thick, sits on face A.
  - plate B: 43.6 x 93.6 mm rounded pad, 12 mm thick, sits on face B.
  - strut: box beam from the centre of plate A to the centre of plate B.
Every bolt has a clearance hole and a counterbore so the ISO 4762 head sits
flush in the pad and the hex key has the space above it.

Everything is built in an invisible Rhino document, in millimetres.
"""
import json
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg
from System.Collections.Generic import List

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TOL = 0.01
UP = (0.0, 0.0, 1.0)

# plate and hole dimensions (mm)
PLATE_A_R = 36.0       # outline of face A is 37.25 radius, keep 1 mm inside
PLATE_A_T = 11.0       # top of plate A = top of the M5 head (floor 6 + head 5)
PLATE_B_HW = 21.8      # outline of face B is 23 half width
PLATE_B_HH = 46.8      # outline of face B is 48 half height
PLATE_B_CORNER = 6.0
PLATE_B_T = 12.0       # top of plate B = top of the M6 head (floor 6 + head 6)
FLOOR = 6.0            # grip: depth of the bottom of the counterbore
HOLE_R = {"A": 2.75, "B": 3.3}      # 5.5 and 6.6 mm clearance holes
CBORE_R = {"A": 5.0, "B": 5.5}      # 10 and 11 mm counterbores
STRUT_HALF_V = 9.0     # strut half height (world Z)
STRUT_HALF_L = 8.0     # strut half width, horizontal
STRUT_DEPTH = 5.0      # strut ends sink this far into the pads


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def unit(a):
    n = (dot(a, a)) ** 0.5
    return (a[0] / n, a[1] / n, a[2] / n)


def add(a, b, s=1.0):
    return (a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def brep_list(items):
    lst = List[rg.Brep]()
    for b in items:
        lst.Add(b)
    return lst


def wall_plane(part):
    """Plane lying on the mounting face of `part`; its normal points into our part."""
    face = FACES[part]
    n = unit(tuple(face["normal"]))
    x = unit(cross(UP, n))              # Z x n, so that x x Z = n
    origin = tuple(face["origin"])
    pl = rg.Plane(P(origin), V(x), V(UP))
    return pl, n, origin


def front_box(pl):
    """Half-space in front of a wall (depth >= 0 along the plane normal)."""
    return rg.Box(pl, rg.Interval(-500, 500), rg.Interval(-500, 500), rg.Interval(0, 500)).ToBrep()


def round_pad(pl, r, t):
    return rg.Cylinder(rg.Circle(pl, r), t).ToBrep(True, True)


def rect_pad(pl, hw, hh, corner, t):
    crv = rg.Rectangle3d(pl, rg.Interval(-hw, hw), rg.Interval(-hh, hh)).ToNurbsCurve()
    crv = rg.Curve.CreateFilletCornersCurve(crv, corner, TOL, 0.1)
    if crv is None:
        raise RuntimeError("could not round the plate B outline")
    return rg.Extrusion.Create(crv, t, True).ToBrep()


def cylinder_along(origin, axis, r, length):
    return rg.Cylinder(rg.Circle(rg.Plane(P(origin), V(axis)), r), length).ToBrep(True, True)


def strut(p1, p2):
    d = unit(add(p2, p1, -1.0))
    length = dot(add(p2, p1, -1.0), d)
    yv = unit(add(UP, d, -dot(UP, d)))  # world Z made perpendicular to the strut
    pl = rg.Plane(P(p1), V(d), V(yv))
    return rg.Box(pl, rg.Interval(0, length), rg.Interval(-STRUT_HALF_V, STRUT_HALF_V),
                  rg.Interval(-STRUT_HALF_L, STRUT_HALF_L)).ToBrep()


def clip(a, b):
    res = rg.Brep.CreateBooleanIntersection(brep_list([a]), brep_list([b]), TOL)
    if not res or len(res) != 1:
        raise RuntimeError("intersection did not give one piece")
    return res[0]


def build():
    plA, nA, oA = wall_plane("A")
    plB, nB, oB = wall_plane("B")

    pad_a = round_pad(plA, PLATE_A_R, PLATE_A_T)
    pad_b = rect_pad(plB, PLATE_B_HW, PLATE_B_HH, PLATE_B_CORNER, PLATE_B_T)

    # strut ends sunk into the pads so that its section stays in front of both walls
    p1 = add(oA, nA, 8.0)
    p2 = add(oB, nB, 5.0)
    beam = strut(p1, p2)

    res = rg.Brep.CreateBooleanUnion(brep_list([pad_a, pad_b, beam]), TOL)
    if not res or len(res) != 1:
        raise RuntimeError("union of pads and strut is not one piece")
    body = res[0]

    cutters = []
    for bolt in BOLTS:
        part = bolt["id"][0]
        pos = tuple(bolt["position"])
        axis = unit(tuple(bolt["axis"]))
        r_hole = HOLE_R[part]
        r_cb = CBORE_R[part]
        # clearance hole from behind the wall to the counterbore floor
        cutters.append(cylinder_along(add(pos, axis, -3.0), axis, r_hole, 3.0 + FLOOR))
        # counterbore for the head, from the floor up through the top of the pad
        cutters.append(cylinder_along(add(pos, axis, FLOOR), axis, r_cb, 10.0))

    cut = rg.Brep.CreateBooleanDifference(brep_list([body]), brep_list(cutters), TOL)
    if not cut or len(cut) != 1:
        raise RuntimeError("counterbores and holes did not give one piece")
    final = cut[0]
    final.MergeCoplanarFaces(TOL)
    return final


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        brep = build()
        if not (brep.IsValid and brep.IsSolid):
            raise RuntimeError("the part is not a valid closed solid")
        doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
        bb = brep.GetBoundingBox(True)
        msg = "ok: wrote output.step, volume %.1f mm3, bbox %s to %s\n" % (
            brep.GetVolume(), bb.Min, bb.Max)
    finally:
        doc.Dispose()
    open(LOG, "w").write(msg)


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
