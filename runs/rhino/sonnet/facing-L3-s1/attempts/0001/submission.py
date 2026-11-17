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
HB, RB = 3.8, 5.6   # B arm half width, boss radius
SLOPE = math.radians(50.0)


def solid(b):
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(x, y, z0, z1, r):
    c = rg.Circle(rg.Plane(rg.Point3d(x, y, z0), rg.Vector3d.ZAxis), r)
    return solid(rg.Cylinder(c, z1 - z0).ToBrep(True, True))


def bar(cx, cy, d, length, hw, z0, z1):
    pl = rg.Plane(rg.Point3d(cx, cy, z0), rg.Vector3d(d[0], d[1], 0), rg.Vector3d(-d[1], d[0], 0))
    bx = rg.Box(pl, rg.Interval(0, length), rg.Interval(-hw, hw), rg.Interval(0, z1 - z0))
    return solid(bx.ToBrep())


def web(cx, cy, d, s0, hw):
    z_in = ZB - TB - (s0 - 4.0) * math.tan(SLOPE)
    pl = rg.Plane(rg.Point3d(cx - (-d[1]) * hw, cy - d[0] * hw, 0), rg.Vector3d(d[0], d[1], 0), rg.Vector3d.ZAxis)
    pts = [(0, ZB - TB), (s0, ZB - TB), (4.0, z_in), (0, z_in), (0, ZB - TB)]
    pl_pts = [pl.PointAt(p[0], p[1]) for p in pts]
    crv = rg.PolylineCurve(pl_pts)
    n = rg.Vector3d(-d[1], d[0], 0) * (2 * hw)
    srf = rg.Surface.CreateExtrusion(crv, n)
    b = srf.ToBrep().CapPlanarHoles(TOL)
    return solid(b)


def build():
    bolts = {b["id"]: b["position"] for b in BOLTS}
    A = [bolts[k] for k in ("A1", "A2", "A3", "A4")]
    B = [bolts[k] for k in ("B1", "B2", "B3")]
    ca = [sum(p[i] for p in A) / 4.0 for i in (0, 1)]
    cb = [sum(p[i] for p in B) / 3.0 for i in (0, 1)]
    cx, cy = (ca[0] + cb[0]) / 2.0, (ca[1] + cb[1]) / 2.0
    parts = [cyl(cx, cy, ZA, ZB, RC)]
    holes = []
    for p in A:
        dx, dy = p[0] - cx, p[1] - cy
        L = math.hypot(dx, dy)
        d = (dx / L, dy / L)
        parts.append(bar(cx, cy, d, L, HA, ZA, ZA + TA))
        parts.append(cyl(p[0], p[1], ZA, ZA + TA, RA))
        holes.append(cyl(p[0], p[1], ZA - 1, ZA + TA + 1, 2.25))
    for p in B:
        dx, dy = p[0] - cx, p[1] - cy
        L = math.hypot(dx, dy)
        d = (dx / L, dy / L)
        parts.append(bar(cx, cy, d, L, HB, ZB - TB, ZB))
        parts.append(cyl(p[0], p[1], ZB - TB, ZB, RB))
        parts.append(web(cx, cy, d, L - 5.0, HB))
        holes.append(cyl(p[0], p[1], ZB - TB - 1, ZB + 1, 2.75))
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
    u = [acc]
    r = rg.Brep.CreateBooleanDifference([u[0]], holes, TOL)
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
