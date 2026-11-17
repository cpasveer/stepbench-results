#! python3
"""Connecting part for facing-L1-s1, modelled in Rhino (RhinoCommon).

Design: a bridge between the two facing mounts.
  * foot plate on mount A (rounded square 69 mm, 8 mm thick), 4 M4 clearance
    holes; heads sit on top of the plate, so the space above the plate and
    around the heads stays open for the L-key.
  * top plate on mount B (disc 81 mm dia, 8 mm thick), 3 M5 clearance holes;
    heads hang below the plate.
  * central column (24 mm dia) joining the two plates, with 5 mm fillets at
    both ends. The column sits at the middle of the bolt pattern, so every
    key sector (pointing outward from each bolt) stays clear.

Writes output.step next to this file; messages go to submission.log.
"""
import json
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}

# Mount A: flat square face, origin and in-plane axes from task.json placements.
A_ORIGIN = (-126.0, 127.5, -94.0)
A_X = (0.5735765984126814, -0.8191521213976911, 0.0)
A_Y = (0.8191521213976911, 0.5735765984126814, 0.0)
A_Z_FACE = -94.0
# Mount B: flat round face at z = -25.5 (body above it, normal -Z).
B_CENTER = (-124.18164228311065, 135.11206773573574)
B_Z_FACE = -25.5

T_A = 8.0            # foot plate thickness = grip for the M4 bolts
T_B = 8.0            # top plate thickness = grip for the M5 bolts
A_HALF = 34.5        # half side of the square foot plate (face is 73 mm)
A_CORNER_R = 6.0
B_PLATE_R = 40.5     # disc radius (face radius 42.25, keep 1 mm inside)
COL_R = 12.0         # column radius
COL_FILLET = 5.0
HOLE_A_R = 2.25      # M4 clearance 4.5 mm
HOLE_B_R = 2.75      # M5 clearance 5.5 mm

Z_A_TOP = A_Z_FACE + T_A        # -86
Z_B_BOT = B_Z_FACE - T_B        # -33.5


def vec(a):
    return rg.Vector3d(a[0], a[1], a[2])


def pt(a):
    return rg.Point3d(a[0], a[1], a[2])


def cylinder(x, y, z0, r, h):
    pl = rg.Plane(rg.Point3d(x, y, z0), rg.Vector3d.ZAxis)
    return rg.Cylinder(rg.Circle(pl, r), h).ToBrep(True, True)


def foot_plate():
    pl = rg.Plane(pt(A_ORIGIN), vec(A_X), vec(A_Y))
    rect = rg.Rectangle3d(pl, rg.Interval(-A_HALF, A_HALF),
                          rg.Interval(-A_HALF, A_HALF)).ToNurbsCurve()
    crv = rg.Curve.CreateFilletCornersCurve(rect, A_CORNER_R, 0.01, 0.1)
    if crv is None:
        raise RuntimeError("could not round foot plate corners")
    brep = rg.Extrusion.Create(crv, T_A, True).ToBrep()
    bb = brep.GetBoundingBox(True)
    if abs(bb.Min.Z - A_Z_FACE) > 1e-3 or abs(bb.Max.Z - Z_A_TOP) > 1e-3:
        raise RuntimeError("foot plate extruded the wrong way: %s" % bb)
    return brep


def top_plate():
    x, y = B_CENTER
    return cylinder(x, y, Z_B_BOT, B_PLATE_R, T_B)


def column():
    cx = 0.5 * (A_ORIGIN[0] + B_CENTER[0])
    cy = 0.5 * (A_ORIGIN[1] + B_CENTER[1])
    # run a little into both plates so the union is robust
    return cylinder(cx, cy, Z_A_TOP - 4.0, COL_R, (Z_B_BOT + 4.0) - (Z_A_TOP - 4.0))


def unite(parts):
    res = rg.Brep.CreateBooleanUnion(parts, 0.001)
    if not res or len(res) != 1:
        raise RuntimeError("union did not give one piece: %d" % (len(res) if res else 0))
    return res[0]


def fillet_column_joints(brep):
    cx = 0.5 * (A_ORIGIN[0] + B_CENTER[0])
    cy = 0.5 * (A_ORIGIN[1] + B_CENTER[1])
    idx = []
    for i, e in enumerate(brep.Edges):
        d = e.Domain
        samples = [e.PointAt(d.Min), e.PointAt(0.5 * (d.Min + d.Max)), e.PointAt(d.Max)]
        ok = True
        for p in samples:
            r = ((p.X - cx) ** 2 + (p.Y - cy) ** 2) ** 0.5
            if abs(r - COL_R) > 0.01:
                ok = False
            if not (abs(p.Z - Z_A_TOP) < 0.01 or abs(p.Z - Z_B_BOT) < 0.01):
                ok = False
        if ok:
            idx.append(i)
    if not idx:
        raise RuntimeError("no column joint edges found")
    radii = [COL_FILLET] * len(idx)
    out = rg.Brep.CreateFilletEdges(brep, idx, radii, radii, rg.BlendType.Fillet,
                                    rg.RailType.DistanceFromEdge, 0.001)
    if not out or len(out) != 1 or not out[0].IsValid:
        raise RuntimeError("fillet failed")
    return out[0]


def bolt_holes(brep):
    cutters = []
    for bid in ("A1", "A2", "A3", "A4"):
        p = BOLTS[bid]["position"]
        cutters.append(cylinder(p[0], p[1], A_Z_FACE - 1.0, HOLE_A_R, T_A + 2.0))
    for bid in ("B1", "B2", "B3"):
        p = BOLTS[bid]["position"]
        cutters.append(cylinder(p[0], p[1], Z_B_BOT - 1.0, HOLE_B_R, T_B + 2.0))
    res = rg.Brep.CreateBooleanDifference([brep], cutters, 0.001)
    if not res or len(res) != 1:
        raise RuntimeError("hole cut did not give one piece")
    return res[0]


def build():
    body = unite([foot_plate(), top_plate(), column()])
    try:
        body = fillet_column_joints(body)
        fillet_note = "fillets ok"
    except Exception as ex:  # keep the unfilleted body if the blend fails
        fillet_note = "fillets skipped (%s)" % ex
    body = bolt_holes(body)
    return body, fillet_note


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        body, note = build()
        if not body.IsValid or not body.IsSolid:
            raise RuntimeError("result is not a valid closed solid")
        vol = rg.VolumeMassProperties.Compute(body).Volume
        bb = body.GetBoundingBox(True)
        doc.Objects.AddBrep(body)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step; volume %.1f mm3; bbox z %.2f..%.2f; %s\n"
                         % (vol, bb.Min.Z, bb.Max.Z, note))


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
