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
    plateB = prism(pB, [(-21, -46), (21, -46), (21, 46), (-21, 46)], TB)
    # bridge: plan polygon extruded in z
    hw = 7.0
    def frontpt(c, n, t, th, u):
        return c + n * th + t * u
    LA = frontpt(cA, nA, tA, TA, -hw); RA = frontpt(cA, nA, tA, TA, hw)
    LB = frontpt(cB, nB, tB, TB, -hw); RB = frontpt(cB, nB, tB, TB, hw)
    # B tangent runs opposite to A's; pair points by side in plan
    # side 1: A(-hw) with B(+hw)?  choose the pairing that does not cross
    def cross2(a, b):
        return a.X * b.Y - a.Y * b.X
    # decide using direction from A to B
    d = cB - cA
    sA = cross2(d, tA)
    sB = cross2(d, tB)
    sides = []
    for sgn in (-1, 1):
        a = frontpt(cA, nA, tA, TA, sgn * hw)
        # matching B point on the same side of the A->B line
        for ub in (-hw, hw):
            b = frontpt(cB, nB, tB, TB, ub)
        sides.append(a)
    # simpler: pair by lateral coordinate along d-perp
    perp = rg.Vector3d(-d.Y, d.X, 0); perp.Unitize()
    ptsA = sorted([LA, RA], key=lambda p: (p - cA) * perp)
    ptsB = sorted([LB, RB], key=lambda p: (p - cB) * perp)
    ext = 3.0
    def extend(p, q):
        dv = q - p; dv.Unitize()
        return p - dv * ext, q + dv * ext
    a1, b1 = extend(ptsA[0], ptsB[0])
    a2, b2 = extend(ptsA[1], ptsB[1])
    z0, z1 = -24.0, 0.0
    pl = rg.Plane(rg.Point3d(0, 0, z0), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    poly = [(a1.X, a1.Y), (b1.X, b1.Y), (b2.X, b2.Y), (a2.X, a2.Y)]
    # ensure counter-clockwise irrelevant; build
    bridge = prism(pl, poly, z1 - z0)
    parts = rg.Brep.CreateBooleanUnion([plateA, plateB, bridge], tol)
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
    body.MergeCoplanarFaces(tol)
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
    open(LOG, "w").write("ok: wrote output.step\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
