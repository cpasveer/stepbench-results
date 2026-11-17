#! python3
"""Connecting part between round flange A and rectangular block B (flange-L2-s1).

Design (world mm):
  * seat disc on A: dia 70, 6 thick, sits inside the 74.5 face outline,
    six 5.5 mm clearance holes on the bolt circle.
  * seat pad on B: 44 x 94 x 6 with rounded corners, inside the 46 x 96 face,
    four 6.6 mm clearance holes on the 30 x 80 grid.
  * one solid rib (rectangular bar) joining the two pads, its ends trimmed
    to the seat planes so nothing goes behind a mounting face.
The script builds the part in its own headless document, checks a few
requirements itself (key-turning sweep, seat planes, bolt sizes) and writes
output.step. Messages go to submission.log.
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
POSE = TASK["poses"][0]
RES = POSE["resolved"]
PLACE = POSE["placements"]
TOL = 0.001

# ---- design parameters -------------------------------------------------
A_DISC_R = 35.0       # inside 37.25 face radius, 1 mm setback respected
A_THICK = 6.0         # grip for M5
B_HALF_X = 22.0       # inside 23 mm half width
B_HALF_Y = 47.0       # inside 48 mm half height
B_CORNER_R = 5.0
B_THICK = 6.0         # grip for M6
RIB_W = 16.0          # rib width (lateral)
RIB_T = 10.0          # rib thickness
DZ_A = 14.0           # rib attaches to A disc this far above the centre
DZ_B = -30.0          # rib attaches to B pad this far below the centre
PA_IN = 3.0           # rib start depth inside A disc (along nA)
PB_IN = 3.0           # rib end depth inside B pad (along nB)
HOLE_DIA = {"M5": 5.5, "M6": 6.6}


def pt(a):
    return rg.Point3d(a[0], a[1], a[2])


def vec(a):
    v = rg.Vector3d(a[0], a[1], a[2])
    v.Unitize()
    return v


def face(part):
    f = [x for x in RES["mounting_faces"] if x["part"] == part][0]
    R = PLACE[part]["rotation"]
    origin = pt(f["origin"])
    x = rg.Vector3d(R[0][0], R[1][0], R[2][0])
    y = rg.Vector3d(R[0][1], R[1][1], R[2][1])
    pl = rg.Plane(origin, x, y)
    n = vec(f["normal"])
    if (pl.Normal * n) < 0.999:
        raise RuntimeError("face frame does not match normal for " + part)
    return pl


def bolts(part):
    return [b for b in RES["bolts"] if b["id"].startswith(part)]


def solid_parts(breps):
    if len(breps) != 1:
        raise RuntimeError("expected one brep, got %d" % len(breps))
    return breps[0]


def build():
    plA = face("A")
    plB = face("B")
    nA = plA.Normal
    nB = plB.Normal
    OA = plA.Origin
    OB = plB.Origin

    # A seat disc
    discA = rg.Cylinder(rg.Circle(plA, A_DISC_R), A_THICK).ToBrep(True, True)

    # B seat pad with rounded corners
    rect = rg.Rectangle3d(plB, rg.Interval(-B_HALF_X, B_HALF_X),
                          rg.Interval(-B_HALF_Y, B_HALF_Y)).ToNurbsCurve()
    rounded = rg.Curve.CreateFilletCornersCurve(rect, B_CORNER_R, TOL, 0.1)
    if rounded is None:
        rounded = rect
    padB = rg.Extrusion.Create(rounded, B_THICK, True).ToBrep()

    # rib between the two pads
    PA = OA + nA * PA_IN + rg.Vector3d(0, 0, DZ_A)
    PB = OB + nB * PB_IN + rg.Vector3d(0, 0, DZ_B)
    axis = PB - PA
    L = axis.Length
    axis.Unitize()
    w = rg.Vector3d.CrossProduct(axis, nA)
    w.Unitize()
    t = rg.Vector3d.CrossProduct(axis, w)
    bar_plane = rg.Plane(PA, w, t)
    box = rg.Box(bar_plane, rg.Interval(-RIB_W / 2, RIB_W / 2),
                 rg.Interval(-RIB_T / 2, RIB_T / 2), rg.Interval(0, L))
    rib = box.ToBrep()
    # trim the rib to the front of both mounting faces
    fa = rg.Box(rg.Plane(OA, nA), rg.Interval(-300, 300), rg.Interval(-300, 300),
                rg.Interval(0, 300)).ToBrep()
    fb = rg.Box(rg.Plane(OB, nB), rg.Interval(-300, 300), rg.Interval(-300, 300),
                rg.Interval(0, 300)).ToBrep()
    rib = solid_parts(rg.Brep.CreateBooleanIntersection([rib], [fa], TOL))
    rib = solid_parts(rg.Brep.CreateBooleanIntersection([rib], [fb], TOL))

    body = solid_parts(rg.Brep.CreateBooleanUnion([discA, padB, rib], TOL))

    # clearance holes along every bolt axis
    cutters = []
    for b in RES["bolts"]:
        ax = vec(b["axis"])
        pos = pt(b["position"])
        d = HOLE_DIA[b["size"]]
        start = pos - ax * 1.0
        cutters.append(rg.Cylinder(rg.Circle(rg.Plane(start, ax), d / 2), 10.0).ToBrep(True, True))
    res = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    body = solid_parts(res)
    body.MergeCoplanarFaces(TOL)
    return body


def key_sweep_report(body, part):
    """For each bolt of this part, find free 60 degree sectors of the L-key sweep.

    The long key leg sits about head top + 20 mm above the face, so the swept
    slab is taken from head top + 3 to head top + 30 out to radius 72.
    """
    lines = []
    for b in bolts(part):
        ax = vec(b["axis"])
        pos = pt(b["position"])
        head_top = (A_THICK if part == "A" else B_THICK) + b["head_height"]
        blocked = [False] * 360
        h = head_top + 3.0
        while h <= head_top + 30.0:
            orig = pos + ax * h
            pl = rg.Plane(orig, ax)
            r = 2.0
            while r <= 72.0:
                deg = 0
                while deg < 360:
                    a = deg * 3.141592653589793 / 180.0
                    p = pl.Origin + pl.XAxis * (r * math.cos(a)) + pl.YAxis * (r * math.sin(a))
                    if body.IsPointInside(p, TOL, False):
                        blocked[deg] = True
                    deg += 4
                r += 2.5
            h += 4.0
        free = []
        for s in range(360):
            ok = True
            for k in range(0, 61, 2):
                if blocked[(s + k) % 360]:
                    ok = False
                    break
            if ok:
                free.append(s)
        lines.append("key %s: free 60 deg sectors start at %s" % (b["id"], (free[:1] if free else "NONE")))
    return lines


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    report = []
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        body = build()
        if not body.IsSolid or not body.IsValid:
            raise RuntimeError("body is not a valid closed solid")
        report.append("volume %.1f mm3" % body.GetVolume())
        bb = body.GetBoundingBox(True)
        report.append("bbox min %s max %s" % (bb.Min, bb.Max))
        report.extend(key_sweep_report(body, "A"))
        report.extend(key_sweep_report(body, "B"))
        doc.Objects.AddBrep(body)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(report) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
