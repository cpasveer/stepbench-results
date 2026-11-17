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


def stadium(p0, p1, r, z):
    import math
    dx, dy = p1[0]-p0[0], p1[1]-p0[1]
    L = math.hypot(dx, dy); ux, uy = dx/L, dy/L; nx, ny = -uy, ux
    P = lambda p, a, b: rg.Point3d(p[0]+a*ux+b*nx, p[1]+a*uy+b*ny, z)
    pc = rg.PolyCurve()
    pc.Append(rg.Line(P(p0, 0, -r), P(p1, 0, -r)))
    pc.Append(rg.Arc(P(p1, 0, -r), P(p1, r, 0), P(p1, 0, r)))
    pc.Append(rg.Line(P(p1, 0, r), P(p0, 0, r)))
    pc.Append(rg.Arc(P(p0, 0, r), P(p0, -r, 0), P(p0, 0, -r)))
    return pc


def plate(center, pts, z0, t, hub_r, arm_r):
    crvs = [rg.Circle(rg.Plane(rg.Point3d(center[0], center[1], z0), rg.Vector3d.ZAxis), hub_r).ToNurbsCurve()]
    for p in pts:
        crvs.append(stadium(center, p, arm_r, z0))
    u = rg.Curve.CreateBooleanUnion(crvs, 0.001)
    assert len(u) == 1, len(u)
    return rg.Extrusion.Create(u[0], t, True).ToBrep()


def build():
    tol = 0.001
    A = [b for b in BOLTS if b["id"].startswith("A")]
    B = [b for b in BOLTS if b["id"].startswith("B")]
    za = FACES[0]["origin"][2]; zb = FACES[1]["origin"][2]
    t = 6.0
    ca = FACES[0]["origin"]; cb = FACES[1]["origin"]
    mid = ((ca[0]+cb[0])/2, (ca[1]+cb[1])/2)
    lower = plate(mid, [b["position"] for b in A], za, t, 12.0, 5.0)
    upper = plate(mid, [b["position"] for b in B], zb - t, t, 12.0, 5.0)
    col = rg.Cylinder(rg.Circle(rg.Plane(rg.Point3d(mid[0], mid[1], za + t - 1), rg.Vector3d.ZAxis), 9.0), (zb - t + 1) - (za + t - 1)).ToBrep(True, True)
    r = rg.Brep.CreateBooleanUnion([lower, col, upper], tol)
    body = r[0]
    cuts = []
    for b in BOLTS:
        d = 4.5 if b["size"] == "M4" else 5.5
        p = b["position"]
        z = za - 1 if b["id"].startswith("A") else zb - t - 1
        cuts.append(rg.Cylinder(rg.Circle(rg.Plane(rg.Point3d(p[0], p[1], z), rg.Vector3d.ZAxis), d/2), t + 2).ToBrep(True, True))
    r = rg.Brep.CreateBooleanDifference([body], cuts, tol)
    br = r[0]
    br.MergeCoplanarFaces(0.001)
    info = [(f.SurfaceIndex, f.UnderlyingSurface().GetType().Name) for f in br.Faces]
    open(os.path.join(HERE, 'scratch_info.txt'), 'w').write(str(info) + str(br.Faces.Count))
    return [br]


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
