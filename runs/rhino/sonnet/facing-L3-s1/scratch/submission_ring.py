#! python3
"""Your answer to this task, modelled in Rhino.

Write ONE solid to output.step (next to this file) that bolts to the fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely.

    sh run_rhino.sh                                  # runs this script inside Rhino, writes output.step
    jointbench verify . --by <your name> --tool rhino

The script builds the part in its own invisible Rhino document (millimetres,
world coordinates) and exports it as STEP. Its messages and any error go to
submission.log; on an error no output.step is left behind.
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
FACES = POSE.get("mounting_faces", [])  # [{"part", "origin", "normal"}]
BOLTS = POSE.get("bolts", [])  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]


import math

TOL = 0.001
ZA = -94.0
ZB = -25.5
TA = 4.0      # A flange thickness
TB = 4.0      # B flange thickness
RC = 9.0      # column radius
HA, RA = 3.6, 5.2   # A arm half width, boss radius
HB, WB, GB = 3.8, 6.65, 4.85   # B arm half width, wide half width, groove half width
SLOPE = math.radians(46.0)


def solid(b):
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(x, y, z0, z1, r, xdir=(1.0, 0.0)):
    pl = rg.Plane(rg.Point3d(x, y, z0), rg.Vector3d(xdir[0], xdir[1], 0), rg.Vector3d(-xdir[1], xdir[0], 0))
    return solid(rg.Cylinder(rg.Circle(pl, r), z1 - z0).ToBrep(True, True))


def bar(cx, cy, d, length, hw, z0, z1):
    pl = rg.Plane(rg.Point3d(cx, cy, z0), rg.Vector3d(d[0], d[1], 0), rg.Vector3d(-d[1], d[0], 0))
    bx = rg.Box(pl, rg.Interval(0, length), rg.Interval(-hw, hw), rg.Interval(0, z1 - z0))
    return solid(bx.ToBrep())


def prism(plane, pts, w0, w1):
    """polygon pts (u,v) in plane, extruded along plane normal from w0 to w1"""
    P = [plane.PointAt(u, v) + plane.ZAxis * w0 for u, v in pts]
    P.append(P[0])
    crv = rg.PolylineCurve(P)
    srf = rg.Surface.CreateExtrusion(crv, plane.ZAxis * (w1 - w0))
    return solid(srf.ToBrep().CapPlanarHoles(TOL))


def barm(cx, cy, d):
    """B arm: plan polygon extruded, cut by the sloped profile, with a groove for the head."""
    pass


def build():
    bolts = {b["id"]: b["position"] for b in BOLTS}
    A = [bolts[k] for k in ("A1", "A2", "A3", "A4")]
    B = [bolts[k] for k in ("B1", "B2", "B3")]
    ca = [sum(p[i] for p in A) / 4.0 for i in (0, 1)]
    cb = [sum(p[i] for p in B) / 3.0 for i in (0, 1)]
    cx, cy = (ca[0] + cb[0]) / 2.0, (ca[1] + cb[1]) / 2.0
    parts = []
    holes = []
    arm_b = []
    for p in A:
        dx, dy = p[0] - cx, p[1] - cy
        L = math.hypot(dx, dy)
        d = (dx / L, dy / L)
        parts.append(bar(cx, cy, d, L, HA, ZA, ZA + TA))
        parts.append(cyl(p[0], p[1], ZA, ZA + TA, RA, (-d[0], -d[1])))
        holes.append(cyl(p[0], p[1], ZA - 1, ZA + TA + 1, 2.25))
    t = math.tan(SLOPE)
    for p in B:
        dx, dy = p[0] - cx, p[1] - cy
        L = math.hypot(dx, dy)
        d = (dx / L, dy / L)
        s_end = L + 6.0
        s0 = L - 4.85
        sf = s0 - 8.0
        zb = ZB - TB
        # plan prism (u along arm, v lateral), z from ZA+TA to ZB
        planpl = rg.Plane(rg.Point3d(cx, cy, ZA + TA), rg.Vector3d(d[0], d[1], 0), rg.Vector3d(-d[1], d[0], 0))
        plan = [(0, -HB), (sf, -HB), (s0, -WB), (s_end - 2, -WB), (s_end, -WB + 2), (s_end, WB - 2), (s_end - 2, WB), (s0, WB), (sf, HB), (0, HB)]
        pp = prism(planpl, plan, 0.0, ZB - (ZA + TA))
        # side profile prism
        zc = zb - (s_end - 4.0) * t
        sidepl = rg.Plane(rg.Point3d(cx, cy, 0), rg.Vector3d(d[0], d[1], 0), rg.Vector3d.ZAxis)
        prof = [(-3, ZB + 1), (s_end + 3, ZB + 1), (s_end + 3, zb + 3 * t), (4.0, zc), (-3, zc)]
        sp = prism(sidepl, prof, -10.0, 10.0)
        inter = rg.Brep.CreateBooleanIntersection(pp, sp, TOL)
        if not inter or len(inter) != 1:
            raise RuntimeError("intersect failed pp %s %s %.1f sp %s %s %.1f" % (pp.IsValid, pp.IsSolid, pp.GetVolume(), sp.IsValid, sp.IsSolid, sp.GetVolume()))
        arm = inter[0]
        g = cyl(p[0], p[1], ZA, zb, GB, d)
        dres = rg.Brep.CreateBooleanDifference([arm], [g], TOL)
        if not dres or len(dres) != 1:
            raise RuntimeError("pocket failed")
        arm_b.append(dres[0])
        holes.append(cyl(p[0], p[1], zb - 1, ZB + 1, 2.75))
    parts.append(cyl(cx, cy, ZA, ZB, RC, (math.cos(0.3), math.sin(0.3))))
    parts += arm_b
    msg = []
    for i, pb in enumerate(parts):
        msg.append("%d valid=%s solid=%s vol=%.1f" % (i, pb.IsValid, pb.IsSolid, pb.GetVolume()))
    acc = parts[0]
    for i, pb in enumerate(parts[1:], 1):
        u = rg.Brep.CreateBooleanUnion([acc, pb], TOL)
        if not u or len(u) != 1:
            raise RuntimeError("union failed at %d\n%s" % (i, "\n".join(msg)))
        acc = u[0]
        acc.MergeCoplanarFaces(TOL)
    r = rg.Brep.CreateBooleanDifference([acc], holes, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("difference failed")
    b = r[0]
    b.MergeCoplanarFaces(TOL)
    if not b.IsValid or not b.IsSolid:
        raise RuntimeError("invalid brep valid=%s solid=%s" % (b.IsValid, b.IsSolid))
    open(LOG + ".info", "w").write("vol=%.1f faces=%d\n" % (b.GetVolume(), b.Faces.Count))
    return [b]


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
    open(LOG, "w").write("ok: wrote output.step\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
