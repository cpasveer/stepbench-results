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


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


def unit(v):
    v = rg.Vector3d(v)
    v.Unitize()
    return v


def obox(center, ax, ay, az, hx, hy, hz):
    pl = rg.Plane(center, ax, ay)
    b = rg.Box(pl, rg.Interval(-hx, hx), rg.Interval(-hy, hy), rg.Interval(-hz, hz))
    return b.ToBrep()


def union(breps):
    r = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not r:
        raise RuntimeError("union failed")
    return list(r)


def build():
    bolts = {b["id"]: b for b in BOLTS}
    # ---- pipe frame
    O = rg.Point3d(21.0, 136.5, -7.0)
    d = unit(rg.Vector3d(0.25, 0.59, 0.77))
    n1 = unit(V(bolts["A1"]["axis"]))
    n2 = unit(V(bolts["A3"]["axis"]))
    m = unit(n1 + n2)
    e2 = rg.Vector3d.CrossProduct(d, m)
    e2.Unitize()
    R = 34.0
    TOP = 40.0           # flat pad planes at 6 mm above the pipe surface
    phib = math.radians(35.0)
    the = math.radians(50.0)
    HL = 25.0            # half length of saddle along the pipe

    def pt2(r, phi, a=-HL):
        return O + d * a + (m * math.cos(phi) + e2 * math.sin(phi)) * r

    r1 = TOP / math.cos(the - phib)
    rr = TOP / math.cos(phib)
    v0 = pt2(R, -the); v1 = pt2(r1, -the); v2 = pt2(rr, 0.0); v3 = pt2(r1, the); v4 = pt2(R, the)
    pc = rg.PolyCurve()
    pc.Append(rg.Line(v0, v1))
    pc.Append(rg.Line(v1, v2))
    pc.Append(rg.Line(v2, v3))
    pc.Append(rg.Line(v3, v4))
    pc.Append(rg.Arc(v4, pt2(R, 0.0), v0))
    ext = rg.Extrusion.Create(pc, 2 * HL, True)
    saddle = ext.ToBrep()
    # make sure extrusion went the right way (centre must be near O)
    bb = saddle.GetBoundingBox(True)
    if bb.Center.DistanceTo(O) > 60:
        ext = rg.Extrusion.Create(pc, -2 * HL, True)
        saddle = ext.ToBrep()

    # ---- B frame
    f = FACES[0]
    CB = P(f["origin"]); N = unit(V(f["normal"])); Z = rg.Vector3d(0, 0, 1)
    T = rg.Vector3d.CrossProduct(N, Z); T.Unitize()
    def bp(t, z, n):
        return CB + T * t + Z * z + N * n
    PADH = 16.5; PADT = 6.0
    pad = obox(bp(0, 0, PADT / 2), T, Z, N, PADH, PADH, PADT / 2)

    # ---- strap
    z0 = -16.0
    E = bp(0, z0 + 4.04, 2.95)
    P0 = O + d * (-10.0) + m * 41.5
    s = unit(P0 - E)
    E8 = E - s * 8.0
    w = unit(T - s * (T * s))
    h = rg.Vector3d.CrossProduct(s, w); h.Unitize()
    L = E8.DistanceTo(P0)
    mid = (E8 + P0) / 2
    strap = obox(mid, s, w, h, L / 2, 7.0, 5.0)

    body = union([saddle, pad, strap])[0]
    # trim below the B face
    half = obox(bp(0, 0, 100), T, Z, N, 300, 300, 100)
    body = rg.Brep.CreateBooleanIntersection([body], [half], TOL)[0]

    # ---- holes
    cut = []
    for b in BOLTS:
        ax = unit(V(b["axis"]))
        dia = 5.5 if b["size"] == "M5" else 4.5
        base = P(b["position"]) - ax * 3.0
        cyl = rg.Cylinder(rg.Circle(rg.Plane(base, ax), dia / 2), 40.0)
        cut.append(cyl.ToBrep(True, True))
    res = rg.Brep.CreateBooleanDifference([body], cut, TOL)
    body = res[0]
    open(os.path.join(HERE, "scratch", "dbg.txt"), "w").write(
        "n=%d valid=%s solid=%s vol=%s faces=%d\n" % (len(res), body.IsValid, body.IsSolid, body.GetVolume(), body.Faces.Count))
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
