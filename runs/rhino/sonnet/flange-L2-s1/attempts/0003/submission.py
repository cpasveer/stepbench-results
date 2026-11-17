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


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def prism(plane, pts2d, depth):
    pts = [plane.PointAt(u, v) for (u, v) in pts2d]
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    srf = rg.Surface.CreateExtrusion(crv, plane.Normal * depth)
    b = rg.Brep.CreateFromSurface(srf).CapPlanarHoles(0.001)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def build():
    tol = 0.001
    zh = rg.Vector3d(0, 0, 1)
    fa = [f for f in FACES if f["part"] == "A"][0]
    fb = [f for f in FACES if f["part"] == "B"][0]
    cA, nA = rg.Point3d(*fa["origin"]), V(fa["normal"])
    cB, nB = rg.Point3d(*fb["origin"]), V(fb["normal"])
    tA = rg.Vector3d.CrossProduct(zh, nA)   # so that tA x z = nA
    tA = -tA if False else tA
    # choose tangents so that t x z = n
    tA = rg.Vector3d(nA.Y, -nA.X, 0) * -1
    tA = rg.Vector3d(-nA.Y, nA.X, 0)
    tB = rg.Vector3d(-nB.Y, nB.X, 0)
    # check t x z == n  (t=(-ny,nx,0): t x z = (nx*1, ny*1,0)) ok
    TA, TB = 8.0, 8.0
    pA = rg.Plane(cA, tA, zh)
    pB = rg.Plane(cB, tB, zh)
    # plate A: 12-gon with flat bottom (no steep overhangs), inside the face outline
    polyA = [(-20, -30), (20, -30), (30, -15), (35.5, 0), (30, 15), (20, 30),
             (-20, 30), (-30, 15), (-35.5, 0), (-30, -15)]
    plateA = prism(pA, polyA, TA)
    plateB = prism(pB, [(-21, -46), (21, -46), (21, 46), (-21, 46)], TB)  # z -64..28
    # --- tail under plate A (set back 1 mm from the face) so the part prints upright on the bed
    ZB = -64.0
    def A(sv, uv):
        return cA + nA * sv + tA * uv
    # side profile (s, z) with a steep chamfer under the 1 mm set-back so there is no flat ceiling
    prof = [(0, -24.5), (1, -26.0), (1, ZB), (8, ZB), (8, -22.0), (0, -22.0)]
    pts = [A(sv, -20) + rg.Vector3d(0, 0, zv) for (sv, zv) in prof]
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    srf = rg.Surface.CreateExtrusion(crv, tA * 40.0)
    tail = rg.Brep.CreateFromSurface(srf).CapPlanarHoles(0.001)
    if tail.SolidOrientation == rg.BrepSolidOrientation.Inward:
        tail.Flip()
    # --- bridge wall: along B's normal, tapered, extruded full height (prints without support)
    cBu, hwA, hwB = 2.0, 4.5, 6.5
    Bf = cB + nB * TB + tB * cBu
    lam = (TA - (Bf - cA) * nA) / (nB * nA)
    # point on centre line at A front (s = TA): Bf + nB*lam
    sides = []
    for sg in (-1, 1):
        qb = Bf + tB * (sg * hwB)
        qa = Bf + nB * lam + tB * (sg * hwA)
        dv = qa - qb; dv.Unitize()
        sides.append((qb - dv * 3.0, qa + dv * 3.0))
    p1, p2 = sides[0]
    p4, p3 = sides[1]
    ztop = 28.0
    plz = rg.Plane(rg.Point3d(0, 0, ZB), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    poly = [(p1.X, p1.Y), (p2.X, p2.Y), (p3.X, p3.Y), (p4.X, p4.Y)]
    bridge = prism(plz, poly, ztop - ZB)
    # clearance report: bridge corners against the bolt axes (in A frame)
    rep = []
    for pt in (p1, p2, p3, p4):
        rep.append("s=%.1f u=%.1f" % ((pt - cA) * nA, (pt - cA) * tA))
    global INFO
    INFO = "bridge corners: " + "; ".join(rep)
    parts = rg.Brep.CreateBooleanUnion([plateA, plateB, bridge, tail], tol)
    body = parts[0]
    cyls = []
    for b in BOLTS:
        lo, hi = b["clearance_diameter_range"]
        r = (lo + hi) / 2.0 / 2.0
        ax = V(b["axis"]); pos = rg.Point3d(*b["position"])
        start = pos - ax * 2.0
        cyl = rg.Cylinder(rg.Circle(rg.Plane(start, ax), r), 30.0)
        cyls.append(cyl.ToBrep(True, True))
    res = rg.Brep.CreateBooleanDifference([body], cyls, tol)
    body = res[0]
    n0 = body.Faces.Count
    for _ in range(3):
        body.MergeCoplanarFaces(tol)
    global INFO2
    INFO2 = "faces %d -> %d" % (n0, body.Faces.Count)
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
    open(LOG, "w").write("ok: wrote output.step\n" + globals().get("INFO", "") + "\n" + globals().get("INFO2", "") + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
