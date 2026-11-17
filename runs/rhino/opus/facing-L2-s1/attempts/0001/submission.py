#! python3
"""Spool adapter between mount A (square, below) and mount B (round, above).

Printed upright, A face on the bed. A hollow, slightly oblique column rises
from a square A flange and flares at 52 deg into a round B flange. The B screws
are put in from below through counterbores in the flare; the A screws are put
in from above onto the flange. Every downward face except the small screw-head
seats is at least 45 deg from horizontal.
"""
import json
import math
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
RES = POSE["resolved"]
BOLTS = RES["bolts"]
TOL = 0.001
msgs = []

cA = rg.Point3d(*POSE["placements"]["A"]["origin"])
cB = rg.Point3d(*POSE["placements"]["B"]["origin"])
RA = POSE["placements"]["A"]["rotation"]
xA = rg.Vector3d(RA[0][0], RA[1][0], 0)  # A local x axis in world
yA = rg.Vector3d(RA[0][1], RA[1][1], 0)

zA = cA.Z            # -94, A face (part sits above)
zB = cB.Z            # -25.5, B face (part sits below)

T_A = 6.0            # A flange thickness (grip A)
T_B = 6.0            # B seat thickness (grip B)
FLANGE = 73.0 - 2 * 1.2
CORNER_R = 5.0
RIM_R = 84.5 / 2 - 1.2
RIM_H = 4.0
CONE_ANG = math.radians(52)
COL_TOP_R = 24.0
COL_BASE_R = 30.0
FILLET = 4.0         # 45 deg chamfer at column foot
WALL = 4.0
HOLE_A = 4.5
HOLE_B = 5.5
POCKET_R = 5.25


def at(c, z):
    return rg.Point3d(c.X, c.Y, z)


def circle(c, z, r):
    pl = rg.Plane(at(c, z), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    return rg.Circle(pl, r).ToNurbsCurve()


def loft(sections):
    crvs = [circle(c, z, r) for (c, z, r) in sections]
    b = rg.Brep.CreateFromLoft(crvs, rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Straight, False)[0]
    b = b.CapPlanarHoles(TOL)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(c, z0, z1, r):
    pl = rg.Plane(at(c, z0), rg.Vector3d.ZAxis)
    return rg.Cylinder(rg.Circle(pl, r), z1 - z0).ToBrep(True, True)


def union(a, bs):
    r = rg.Brep.CreateBooleanUnion([a] + bs, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("union failed: %s" % (None if r is None else len(r)))
    return r[0]


def diff(a, bs):
    r = rg.Brep.CreateBooleanDifference([a], bs, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("difference failed: %s" % (None if r is None else len(r)))
    return r[0]


def build():
    z_flange_top = zA + T_A
    z_col_foot = z_flange_top + FILLET
    z_rim_bot = zB - RIM_H
    z_cone_bot = z_rim_bot - (RIM_R - COL_TOP_R) * math.tan(CONE_ANG)
    foot_r = COL_BASE_R + FILLET

    outer = loft([
        (cA, zA + 1.0, foot_r),
        (cA, z_flange_top, foot_r),
        (cA, z_col_foot, COL_BASE_R),
        (cB, z_cone_bot, COL_TOP_R),
        (cB, z_rim_bot, RIM_R),
        (cB, zB, RIM_R),
    ])

    # square A flange with rounded corners
    pl = rg.Plane(at(cA, zA), xA, yA)
    rect = rg.Rectangle3d(pl, rg.Interval(-FLANGE / 2, FLANGE / 2), rg.Interval(-FLANGE / 2, FLANGE / 2))
    rc = rg.Curve.CreateFilletCornersCurve(rect.ToNurbsCurve(), CORNER_R, TOL, math.radians(1))
    flange = rg.Surface.CreateExtrusion(rc, rg.Vector3d(0, 0, T_A)).ToBrep().CapPlanarHoles(TOL)
    if flange.SolidOrientation == rg.BrepSolidOrientation.Inward:
        flange.Flip()

    body = union(outer, [flange])

    bore_base_r = COL_BASE_R - WALL
    bore_top_r = COL_TOP_R - WALL
    bore = loft([
        (cA, zA - 1.0, bore_base_r),
        (cA, z_col_foot, bore_base_r),
        (cB, z_cone_bot, bore_top_r),
        (cB, zB + 1.0, bore_top_r),
    ])
    cutters = [bore]
    for b in BOLTS:
        p = rg.Point3d(*b["position"])
        if b["axis"][2] > 0:   # A screws, heads on top of the flange
            cutters.append(cyl(p, zA - 1, zA + T_A + 1, HOLE_A / 2))
        else:                  # B screws, heads under the flare in a counterbore
            cutters.append(cyl(p, zB - T_B - 1, zB + 1, HOLE_B / 2))
            cutters.append(cyl(p, z_cone_bot - 12, zB - T_B, POCKET_R))
    body = diff(body, cutters)
    body.MergeCoplanarFaces(TOL)
    msgs.append("cone bottom z=%.2f, solid=%s, valid=%s, faces=%d, volume=%.0f" % (
        z_cone_bot, body.IsSolid, body.IsValid, body.Faces.Count, body.GetVolume()))
    return [body]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build():
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(msgs) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(msgs))
    if os.path.exists(OUT):
        os.remove(OUT)
