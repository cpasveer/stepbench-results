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
T = 8.0  # plate thickness


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


def unit(v):
    v = rg.Vector3d(v)
    v.Unitize()
    return v


def rounded_rect(plane, hx, hy, r):
    segs = []
    def pt(x, y):
        return plane.PointAt(x, y)
    pts = [(hx - r, -hy), (hx, -hy + r), (hx, hy - r), (hx - r, hy),
           (-hx + r, hy), (-hx, hy - r), (-hx, -hy + r), (-hx + r, -hy)]
    pc = rg.PolyCurve()
    c = [(hx - r, -hy + r), (hx - r, hy - r), (-hx + r, hy - r), (-hx + r, -hy + r)]
    for k in range(4):
        a, b = pts[2 * k], pts[2 * k + 1]
        # arc corner k : from a(end of straight) to b
        pass
    # build sequentially: straight bottom, arc, straight right, arc ...
    order = [
        ("L", (-hx + r, -hy), (hx - r, -hy)),
        ("A", (hx - r, -hy), (hx - r + r * math.cos(math.pi / 4), -hy + r - r * math.sin(math.pi / 4)), (hx, -hy + r)),
        ("L", (hx, -hy + r), (hx, hy - r)),
        ("A", (hx, hy - r), (hx - r + r * math.cos(math.pi / 4), hy - r + r * math.sin(math.pi / 4)), (hx - r, hy)),
        ("L", (hx - r, hy), (-hx + r, hy)),
        ("A", (-hx + r, hy), (-hx + r - r * math.cos(math.pi / 4), hy - r + r * math.sin(math.pi / 4)), (-hx, hy - r)),
        ("L", (-hx, hy - r), (-hx, -hy + r)),
        ("A", (-hx, -hy + r), (-hx + r - r * math.cos(math.pi / 4), -hy + r - r * math.sin(math.pi / 4)), (-hx + r, -hy)),
    ]
    for o in order:
        if o[0] == "L":
            pc.Append(rg.LineCurve(pt(*o[1]), pt(*o[2])))
        else:
            pc.Append(rg.ArcCurve(rg.Arc(pt(*o[1]), pt(*o[2]), pt(*o[3]))))
    return pc


def prism(curve, vec):
    srf = rg.Surface.CreateExtrusion(curve, vec)
    b = srf.ToBrep()
    b = b.CapPlanarHoles(0.001)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(origin, axis, r, d0, d1):
    pl = rg.Plane(origin + axis * d0, axis)
    return rg.Cylinder(rg.Circle(pl, r), d1 - d0).ToBrep(True, True)


def union(bs):
    r = rg.Brep.CreateBooleanUnion(bs, 0.001)
    if not r or len(r) != 1:
        raise RuntimeError("union failed %s" % (None if not r else len(r)))
    return r[0]


LOGS = []


def build():
    nA = unit(V(FACES[0]["normal"])); oA = P(FACES[0]["origin"])
    nB = unit(V(FACES[1]["normal"])); oB = P(FACES[1]["origin"])
    up = rg.Vector3d(0, 0, 1)
    tA = rg.Vector3d.CrossProduct(up, nA)  # horizontal tangent
    tB = rg.Vector3d.CrossProduct(up, nB)
    # plate A: disc r=35
    plA = rg.Plane(oA, tA, up)
    if plA.ZAxis * nA < 0:
        plA = rg.Plane(oA, up, tA)
    discA = rg.Cylinder(rg.Circle(rg.Plane(oA, nA), 35.0), T).ToBrep(True, True)
    # plate B: rounded rect 43 x 93
    plB = rg.Plane(oB, tB, up)
    if plB.ZAxis * nB < 0:
        plB = rg.Plane(oB, up, tB)
    cB = rounded_rect(plB, 21.5, 46.5, 5.0)
    plateB = prism(cB, nB * T)
    # bridge bar along nB, z 19..28 (above leg sweep), width 10
    w = 5.0
    dAB = (oB - oA) * nA  # distance along nA between faces
    cosang = (-nB) * nA
    sB0 = 4.0
    sA = (dAB - 4.0) / (-nB * nA * -1) if False else None
    # centre line start inside B plate (depth 4), end inside A plate (depth 4)
    p0 = oB + nB * sB0
    # find s so that (p0 + nB*s - oA).nA = 4
    k = nB * nA
    s = (4.0 - (p0 - oA) * nA) / k
    p1 = p0 + nB * s
    d = nB
    side = rg.Vector3d.CrossProduct(up, d)
    side.Unitize()
    z0, z1 = 19.0, 28.0
    q = [p0 + side * w, p1 + side * w, p1 - side * w, p0 - side * w]
    for a in q:
        pass
    pl = rg.Polyline([rg.Point3d(x.X, x.Y, z0) for x in q] + [rg.Point3d(q[0].X, q[0].Y, z0)])
    bar = prism(pl.ToPolylineCurve(), rg.Vector3d(0, 0, z1 - z0))
    LOGS.append("solids %s %s %s" % (discA.SolidOrientation, plateB.SolidOrientation, bar.SolidOrientation))
    body = union([discA, plateB, bar])
    LOGS.append("body bbox %s vol %s" % (body.GetBoundingBox(True), body.GetVolume()))
    # report tangent drift at A
    for dd in (0.0, 8.0, 13.0):
        pass
    cuts = []
    for b in BOLTS:
        o = P(b["position"]); ax = unit(V(b["axis"]))
        dia = 5.5 if b["size"] == "M5" else 6.6
        cuts.append(cyl(o, ax, dia / 2.0, -2.0, T + 2.0))
    res = rg.Brep.CreateBooleanDifference([body], cuts, 0.001)
    if not res or len(res) != 1:
        raise RuntimeError("difference failed")
    out = res[0]
    out.MergeCoplanarFaces(0.001)
    LOGS.append("out bbox %s vol %s solid %s" % (out.GetBoundingBox(True), out.GetVolume(), out.IsSolid))
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(LOGS) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
