#! python3
"""Facing connector: square flange on A, round flange on B, joined by a hollow column.
Built in a headless Rhino document and exported to output.step."""
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
MSG = []

T_A = 6.0          # flange thickness on A (grip)
T_B = 6.0          # flange thickness on B (grip)
HALF_A = 35.25     # half width of square flange (face 36.5 - 1 setback - margin)
CORNER_A = 7.0
R_B = 40.75        # round flange radius (face 42.25 - 1 setback - margin)
R_COL = 22.0       # column outer radius
WALL = 4.0         # column wall
FILLET = 4.0       # column-to-flange fillet
HOLE_M4 = 4.5
HOLE_M5 = 5.5


def P(v):
    return rg.Point3d(v[0], v[1], v[2])


def solid_extrude(curve, plane, height):
    srf = rg.Surface.CreateExtrusion(curve, plane.ZAxis * height)
    b = srf.ToBrep().CapPlanarHoles(TOL)
    if b is None or not b.IsSolid:
        raise RuntimeError("extrusion not solid")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def union(breps):
    r = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("union failed: %s" % (None if r is None else len(r)))
    return r[0]


def diff(a, cutters):
    r = rg.Brep.CreateBooleanDifference([a], cutters, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("difference failed")
    return r[0]


def build():
    pa = POSE["placements"]["A"]
    pb = POSE["placements"]["B"]
    ca = P(pa["origin"])
    cb = P(pb["origin"])
    rA = pa["rotation"]
    xa = rg.Vector3d(rA[0][0], rA[1][0], 0.0)
    ya = rg.Vector3d(rA[0][1], rA[1][1], 0.0)
    za, zb = ca.Z, cb.Z

    # square flange on A
    plA = rg.Plane(ca, xa, ya)
    rect = rg.Rectangle3d(plA, rg.Interval(-HALF_A, HALF_A), rg.Interval(-HALF_A, HALF_A))
    crv = rect.ToNurbsCurve()
    fil = rg.Curve.CreateFilletCornersCurve(crv, CORNER_A, TOL, 0.01)
    flA = solid_extrude(fil, plA, T_A)

    # round flange on B
    plB = rg.Plane(rg.Point3d(cb.X, cb.Y, zb - T_B), rg.Vector3d.ZAxis)
    flB = solid_extrude(rg.Circle(plB, R_B).ToNurbsCurve(), plB, T_B)

    # column, placed between the two mount centres
    cc = rg.Point3d(ca.X + 0.75 * (cb.X - ca.X), ca.Y + 0.75 * (cb.Y - ca.Y), za)  # balances head clearance on A and B
    plC = rg.Plane(cc, rg.Vector3d.ZAxis)
    col = solid_extrude(rg.Circle(plC, R_COL).ToNurbsCurve(), plC, zb - za)

    MSG.append("vols %.0f %.0f %.0f" % (flA.GetVolume(), flB.GetVolume(), col.GetVolume()))
    body = union([union([flA, col]), flB])
    body.MergeCoplanarFaces(TOL)

    # fillet the two column/flange junctions
    try:
        ids = []
        for e in body.Edges:
            c = e.EdgeCurve
            ok, circ = c.TryGetCircle(TOL)
            if ok and abs(circ.Radius - R_COL) < 0.01:
                z = circ.Center.Z
                if abs(z - (za + T_A)) < 0.01 or abs(z - (zb - T_B)) < 0.01:
                    ids.append(e.EdgeIndex)
        if len(ids) == 2:
            fr = rg.Brep.CreateFilletEdges(body, ids, [FILLET] * 2, [FILLET] * 2,
                                           rg.BlendType.Fillet, rg.RailType.RollingBall, TOL)
            if fr and len(fr) == 1 and fr[0].IsSolid and fr[0].IsValid:
                body = fr[0]
                MSG.append("fillets ok")
            else:
                MSG.append("fillet failed, kept sharp")
        else:
            MSG.append("fillet edges found: %d" % len(ids))
    except Exception as ex:
        MSG.append("fillet exception %s" % ex)

    # bore through the column (lightening)
    bore = solid_extrude(rg.Circle(rg.Plane(rg.Point3d(cc.X, cc.Y, za - 1), rg.Vector3d.ZAxis),
                                   R_COL - WALL).ToNurbsCurve(),
                         rg.Plane(rg.Point3d(cc.X, cc.Y, za - 1), rg.Vector3d.ZAxis), zb - za + 2)
    cutters = [bore]
    for b in BOLTS:
        p = P(b["position"])
        d = HOLE_M4 if b["size"] == "M4" else HOLE_M5
        up = b["axis"][2] > 0
        z0 = (za - 1) if up else (zb - T_B - 1)
        h = (T_A if up else T_B) + 2
        base = rg.Plane(rg.Point3d(p.X, p.Y, z0), rg.Vector3d.ZAxis)
        cutters.append(solid_extrude(rg.Circle(base, d / 2).ToNurbsCurve(), base, h))
    body = diff(body, cutters)
    body.MergeCoplanarFaces(TOL)
    if not (body.IsSolid and body.IsValid):
        raise RuntimeError("result not a valid solid")
    MSG.append("volume %.0f mm3" % body.GetVolume())
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(MSG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(MSG))
    if os.path.exists(OUT):
        os.remove(OUT)
