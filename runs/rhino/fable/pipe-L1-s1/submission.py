#! python3
"""Bracket between a round pipe (A, 4x M5) and a flat square mount (B, 4x M4).

One solid: a saddle that follows the pipe, a rounded square flange on B and a
round strut between them, aimed so every bolt keeps a free column for its key.
Everything is over-built past the two contact surfaces and then cut back by the
B face plane and by the pipe cylinder, so the seats are exact.
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
P0 = TASK["poses"][0]
POSE = P0["resolved"]
BOLTS = POSE["bolts"]
TOL = 0.001
MSG = []

# ---- design numbers (mm) -------------------------------------------------
PIPE_R = 34.0
SADDLE_T = 8.0          # saddle wall
SADDLE_HALF_ANG = 50.0  # bolts sit at +-35 deg
SADDLE_HALF_LEN = 27.0  # bolts sit at +-17.5
SPOT_GRIP = 5.5         # material under the M5 heads (flat spot-face)
SPOT_R = 5.5
A_HOLE_R = 2.75
PLATE = 35.0            # flange across (face is 38: 1.5 inside all round)
PLATE_T = 6.0
PLATE_CORNER = 5.0
B_HOLE_R = 2.25
STRUT_R = 6.5
STRUT_U0 = 4.0          # strut foot offset on the flange, clears B1
STRUT_PHI = -8.0        # strut landing angle round the pipe


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def Pt(a):
    return rg.Point3d(a[0], a[1], a[2])


def one(breps, what):
    if breps is None or len(breps) != 1:
        raise RuntimeError("%s gave %s pieces" % (what, None if breps is None else len(breps)))
    return breps[0]


def cyl(base, axis, r, h):
    axis = rg.Vector3d(axis)
    axis.Unitize()
    c = rg.Cylinder(rg.Circle(rg.Plane(base, axis), r), h)
    return c.ToBrep(True, True)


def extrude_solid(curve, vec):
    """Extrude a closed planar curve segment by segment, so lines give planes and arcs cylinders."""
    walls = []
    for seg in curve.DuplicateSegments():
        ok, arc = seg.TryGetArc(TOL)
        if seg.IsLinear(TOL):
            seg = rg.LineCurve(seg.PointAtStart, seg.PointAtEnd)
        elif ok:
            seg = rg.ArcCurve(arc)
        walls.append(rg.Surface.CreateExtrusion(seg, vec).ToBrep())
    shell = one(rg.Brep.JoinBreps(walls, TOL), "extrusion walls")
    solid = shell.CapPlanarHoles(TOL)
    if solid is None or not solid.IsSolid:
        raise RuntimeError("extrusion did not close")
    return solid


def build():
    RA = P0["placements"]["A"]["rotation"]
    C = Pt(P0["placements"]["A"]["origin"])
    d = rg.Vector3d(RA[0][0], RA[1][0], RA[2][0])    # pipe axis
    yl = rg.Vector3d(RA[0][1], RA[1][1], RA[2][1])
    zl = rg.Vector3d(RA[0][2], RA[1][2], RA[2][2])   # middle of the bolt pattern
    OB = Pt(POSE["mounting_faces"][0]["origin"])
    n = V(POSE["mounting_faces"][0]["normal"])
    n.Unitize()
    zw = rg.Vector3d(0, 0, 1)
    u = rg.Vector3d.CrossProduct(zw, n) * -1.0
    u.Unitize()

    # flange on B, over-built 10 mm behind the face
    pl = rg.Plane(OB - n * 10.0, u, zw)
    if pl.Normal * n < 0:
        pl = rg.Plane(OB - n * 10.0, zw, u)
    h = PLATE / 2.0
    rect = rg.Rectangle3d(pl, rg.Interval(-h, h), rg.Interval(-h, h)).ToNurbsCurve()
    prof = rg.Curve.CreateFilletCornersCurve(rect, PLATE_CORNER, TOL, 0.01)
    plate = extrude_solid(prof, n * (10.0 + PLATE_T))

    # saddle on the pipe, over-built 6 mm into the pipe
    sp = rg.Plane(C - d * SADDLE_HALF_LEN, yl, zl)
    if sp.Normal * d < 0:
        raise RuntimeError("saddle plane flipped")
    a = math.radians(SADDLE_HALF_ANG)
    r0, r1 = PIPE_R - 6.0, PIPE_R + SADDLE_T
    arc0 = rg.Arc(rg.Circle(sp, r0), rg.Interval(math.pi / 2 - a, math.pi / 2 + a)).ToNurbsCurve()
    arc1 = rg.Arc(rg.Circle(sp, r1), rg.Interval(math.pi / 2 - a, math.pi / 2 + a)).ToNurbsCurve()
    l0 = rg.LineCurve(arc0.PointAtStart, arc1.PointAtStart)
    l1 = rg.LineCurve(arc0.PointAtEnd, arc1.PointAtEnd)
    sprof = one(rg.Curve.JoinCurves([arc0, l1, arc1, l0], TOL), "saddle profile")
    saddle = extrude_solid(sprof, d * (2 * SADDLE_HALF_LEN))

    # strut
    S = OB + u * STRUT_U0
    ph = math.radians(STRUT_PHI)
    T = C + zl * (PIPE_R * math.cos(ph)) + yl * (PIPE_R * math.sin(ph))
    sd = T - S
    L = sd.Length
    sd.Unitize()
    strut = cyl(S - sd * 4.0, sd, STRUT_R, L + 8.0)
    axis_line = rg.Line(S, T)
    for b in BOLTS:
        p = Pt(b["position"])
        bl = rg.Line(p, p + V(b["axis"]) * 40.0)
        MSG.append("strut clearance to %s axis: %.2f" % (b["id"], bl.MinimumDistanceTo(axis_line) - STRUT_R))

    for nm, b in (("plate", plate), ("saddle", saddle), ("strut", strut)):
        if not b.IsSolid:
            raise RuntimeError(nm + " not solid")
        if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
            b.Flip()

    body = one(rg.Brep.CreateBooleanUnion([plate, saddle, strut], TOL), "union")

    # cut back to the two contact surfaces
    back = rg.Box(rg.Plane(OB, n), rg.Interval(-150, 150), rg.Interval(-150, 150), rg.Interval(-60, 0)).ToBrep()
    pipe = cyl(C - d * 200.0, d, PIPE_R, 400.0)
    body = one(rg.Brep.CreateBooleanDifference([body], [back], TOL), "cut B plane")
    body = one(rg.Brep.CreateBooleanDifference([body], [pipe], TOL), "cut pipe")

    # holes and spot-faces
    cutters = []
    for b in BOLTS:
        p = Pt(b["position"])
        ax = V(b["axis"])
        ax.Unitize()
        if b["id"].startswith("A"):
            cutters.append(cyl(p - ax * 4.0, ax, A_HOLE_R, 4.0 + SPOT_GRIP + 2.0))
            cutters.append(cyl(p + ax * SPOT_GRIP, ax, SPOT_R, 12.0))
        else:
            cutters.append(cyl(p - ax * 4.0, ax, B_HOLE_R, 4.0 + PLATE_T + 6.0))
    for i, c in enumerate(cutters):
        body = one(rg.Brep.CreateBooleanDifference([body], [c], TOL), "hole %d" % i)

    body.MergeCoplanarFaces(TOL)
    body.Compact()
    if not (body.IsValid and body.IsSolid):
        raise RuntimeError("result not a valid solid")
    vol = rg.VolumeMassProperties.Compute(body).Volume
    MSG.append("faces %d edges %d volume %.0f mm3" % (body.Faces.Count, body.Edges.Count, vol))
    short = min(e.GetLength() for e in body.Edges)
    MSG.append("shortest edge %.3f" % short)
    return [body]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.ModelAbsoluteTolerance = TOL
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
    open(LOG, "w").write("error:\n" + err + "\n".join(MSG) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
