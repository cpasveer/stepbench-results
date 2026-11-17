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
import math
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


def V(x, y, z=0.0):
    return rg.Vector3d(x, y, z)


def extrude_poly(pts, vec):
    c = rg.PolylineCurve(pts + [pts[0]])
    srf = rg.Surface.CreateExtrusion(c, vec)
    b = srf.ToBrep()
    return b.CapPlanarHoles(0.001)


def stadium(p0, p1, r, z0, z1):
    """Vertical prism whose plan is a stadium from p0 to p1 with radius r."""
    d = rg.Vector3d(p1[0] - p0[0], p1[1] - p0[1], 0)
    L = d.Length
    d.Unitize()
    n = rg.Vector3d(-d.Y, d.X, 0)
    a = rg.Point3d(p0[0], p0[1], z0)
    b = rg.Point3d(p1[0], p1[1], z0)
    pc = rg.PolyCurve()
    pc.Append(rg.Line(a + n * (-r), b + n * (-r)))
    pc.Append(rg.Arc(b + n * (-r), b + d * r, b + n * r))
    pc.Append(rg.Line(b + n * r, a + n * r))
    pc.Append(rg.Arc(a + n * r, a - d * r, a + n * (-r)))
    srf = rg.Surface.CreateExtrusion(pc, V(0, 0, z1 - z0))
    b = srf.ToBrep().CapPlanarHoles(0.001)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(x, y, z0, z1, r):
    c = rg.Cylinder(rg.Circle(rg.Plane(rg.Point3d(x, y, z0), rg.Vector3d.ZAxis), r), z1 - z0)
    return c.ToBrep(True, True)


def union(parts):
    res = rg.Brep.CreateBooleanUnion(parts, 0.001)
    return res


def build():
    tol = 0.001
    zA = FACES[0]["origin"][2]
    zB = FACES[1]["origin"][2]
    cA = FACES[0]["origin"]
    cB = FACES[1]["origin"]
    cx = 0.5 * (cA[0] + cB[0]); cy = 0.5 * (cA[1] + cB[1])
    RC = 12.0
    tA, tB = 5.0, 6.0
    pieces = [cyl(cx, cy, zA, zB, RC)]
    for b in BOLTS:
        p = b["position"]
        if b["id"].startswith("A"):
            pieces.append(stadium((cx, cy), p, 5.0, zA, zA + tA))
        else:
            r = 5.5
            st = stadium((cx, cy), p, r, zB - 60, zB)
            # profile in arm plane: s along arm, gusset at 45 deg under arm
            d = rg.Vector3d(p[0] - cx, p[1] - cy, 0); d.Unitize()
            n = rg.Vector3d(-d.Y, d.X, 0)
            s1 = 27.0
            sE = math.hypot(p[0] - cx, p[1] - cy) + 8.0
            zb = zB - tB
            prof = [(0, zb), (s1, zb), (s1, zB), (0, zB)]
            prof = [(0, zb - (s1 - RC) - 0.0), (RC, zb - (s1 - RC)), (s1, zb), (sE, zb), (sE, zB), (0, zB)]
            o = rg.Point3d(cx, cy, 0)
            pts = [o + d * s + n * (-r - 1) + rg.Vector3d(0, 0, z) for (s, z) in prof]
            pr = extrude_poly(pts, n * (2 * r + 2))
            inter = rg.Brep.CreateBooleanIntersection(pr, st, tol)
            pieces.append(inter[0])
    dbg = []
    for q in pieces:
        dbg.append((q.IsSolid, round(q.GetVolume(),1), q.SolidOrientation.ToString()))
    open(LOG+'.dbg','w').write(repr(dbg))
    u = union(pieces)
    open(LOG+'.dbg','a').write(' union n=%d' % (len(u) if u else -1))
    body = u[0]
    body.MergeCoplanarFaces(tol)
    cuts = []
    for b in BOLTS:
        p = b["position"]
        dia = 4.5 if b["size"] == "M4" else 5.5
        cuts.append(cyl(p[0], p[1], zA - 5, zB + 5, dia / 2))
    res = rg.Brep.CreateBooleanDifference([body], cuts, tol)
    out = res[0]
    out.MergeCoplanarFaces(tol)
    return [out]


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
