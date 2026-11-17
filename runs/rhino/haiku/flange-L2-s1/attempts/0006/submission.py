#! python3
"""Connecting part between round flange A and rectangular block B (flange-L2-s1).

Design (world mm, build axis = world up so both seats are vertical):
  * seat pad on A: regular hexagon (circumradius 36) in the A face plane, 6 thick,
    inside the 74.5 face outline, six 5.5 mm clearance holes on the bolt circle.
  * seat pad on B: 44 x 94 x 6 with rounded corners, inside the 46 x 96 face,
    four 6.6 mm clearance holes on the 30 x 80 grid.
  * one rib (rectangular bar, 16 x 10) joining the two pads, steep enough to
    print without support; triangular gussets fill the acute crevices where the
    rib meets each pad, so no joint is shallower than 45 degrees.
The script builds the part in its own headless document, checks the key-turning
sweep itself and writes output.step. Messages go to submission.log.
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
A_HEX_R = 36.0       # hexagon circumradius, inside the 37.25 face outline
A_THICK = 6.0        # grip for M5
B_HALF_X = 22.0      # inside 23 mm half width
B_HALF_Y = 47.0      # inside 48 mm half height
B_CORNER_R = 0.0     # sharp corners: a corner radius tangent to the foot leaves a 0 degree crack
B_THICK = 6.0        # grip for M6
RIB_W = 16.0         # rib width (horizontal, across the rib)
RIB_T = 10.0         # rib thickness
DZ_A = 20.0          # rib starts this far above the A centre (inside the pad)
DZ_B = -26.9         # rib ends this far above the B centre (negative = below)
RIB_BX = 10.5        # rib end offset along the B pad's horizontal axis
PAD_DEPTH = 3.0      # how far the rib ends sit inside each pad
GUSSET_LEG = 6.0     # leg length of the triangular crevice fillers
FOOT_HALF_W = 35.0   # foot half width (lateral), 70 mm
FOOT_DEPTH = 60.0    # foot extent in front of the B face
FOOT_T = 6.0         # foot thickness (z from -65 down to -71)
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


def one(breps):
    if breps is None or len(breps) != 1:
        raise RuntimeError("expected one brep, got %s" % (0 if breps is None else len(breps)))
    return breps[0]


def gusset(X0, dir_vec, pad_n, H, depth, s, t, info, name):
    """Triangular filler on one side (s = +1/-1) of the rib where it meets a pad.

    X0: rib end point inside the pad (depth below the front plane), dir_vec: from the
    pad into the rib, pad_n: pad front normal, H: front plane height above the seat.
    """
    lam = (H - depth - s * (RIB_T / 2) * (t * pad_n)) / (dir_vec * pad_n)
    O = X0 + t * (s * RIB_T / 2) + dir_vec * lam
    m = t * s - pad_n * (t * pad_n) * s
    m.Unitize()
    ang = math.degrees(rg.Vector3d.VectorAngle(m, dir_vec))
    info.append("%s side %+d: crevice angle %.1f deg" % (name, s, ang))
    if ang >= 60.0:
        return None
    # triangle lies in the plane spanned by m and dir_vec, extruded across the rib width
    wv = rg.Vector3d.CrossProduct(m, dir_vec)
    wv.Unitize()
    # triangle with apex at O, extruded symmetrically across the rib width
    p0 = O - wv * (RIB_W / 2)
    p1 = p0 + m * GUSSET_LEG
    p2 = p0 + dir_vec * GUSSET_LEG
    poly = rg.PolylineCurve([p0, p1, p2, p0])
    ext = rg.Extrusion.Create(poly, RIB_W, True)
    if ext is None:
        return None
    b = ext.ToBrep()
    if b is None or not b.IsSolid:
        return None
    # make sure the prism spans +-RIB_W/2 about the apex plane (direction of extrusion is not known a priori)
    offset = (b.GetBoundingBox(True).Center - O) * wv
    if offset < -RIB_W / 4:
        b.Transform(rg.Transform.Translation(wv * RIB_W))
    elif offset > RIB_W / 4:
        b.Transform(rg.Transform.Translation(-wv * RIB_W))
    return b


def build(info):
    plA = face("A")
    plB = face("B")
    nA = plA.Normal
    nB = plB.Normal
    OA = plA.Origin
    OB = plB.Origin
    up = rg.Vector3d(0, 0, 1)

    # A seat pad: regular hexagon in the A face plane (x = horizontal, y = up)
    plAu = rg.Plane(OA, rg.Vector3d.CrossProduct(up, nA), up)
    # profile (w, z) about the bolt circle centre: hexagon outline with the lower
    # edge replaced by two 50 degree chamfers so no flat ceiling faces down
    prof = [(36.0, 0.0), (18.5, 30.75), (-18.5, 30.75), (-36.0, 0.0), (-18.5, -30.5),
            (-11.5, -30.5), (0.0, -16.8), (11.5, -30.5), (18.5, -30.5)]
    pts = [plAu.PointAt(x, y) for (x, y) in prof]
    pts.append(pts[0])
    padA = rg.Extrusion.Create(rg.PolylineCurve(pts), A_THICK, True).ToBrep()

    # B seat pad with rounded corners: 44 x 94 inside the 46 x 96 face
    plBu = rg.Plane(OB, plB.XAxis, up)
    rect = rg.Rectangle3d(plBu, rg.Interval(-B_HALF_X, B_HALF_X),
                          rg.Interval(-B_HALF_Y, B_HALF_Y)).ToNurbsCurve()
    rounded = rect
    if B_CORNER_R > 0:
        rounded = rg.Curve.CreateFilletCornersCurve(rect, B_CORNER_R, TOL, 0.1) or rect
    padB = rg.Extrusion.Create(rounded, B_THICK, True).ToBrep()

    # rib between the two pads
    PA = OA + nA * PAD_DEPTH + rg.Vector3d(0, 0, DZ_A)
    PB = OB + nB * PAD_DEPTH + plBu.XAxis * RIB_BX + rg.Vector3d(0, 0, DZ_B)
    axis = PB - PA
    L = axis.Length
    axis.Unitize()
    w = rg.Vector3d.CrossProduct(axis, nA)
    w.Unitize()
    t = rg.Vector3d.CrossProduct(axis, w)
    bar_plane = rg.Plane(PA, w, t)
    rib = rg.Box(bar_plane, rg.Interval(-RIB_W / 2, RIB_W / 2),
                 rg.Interval(-RIB_T / 2, RIB_T / 2), rg.Interval(0, L)).ToBrep()
    fa = rg.Box(rg.Plane(OA, nA), rg.Interval(-300, 300), rg.Interval(-300, 300),
                rg.Interval(0, 300)).ToBrep()
    fb = rg.Box(rg.Plane(OB, nB), rg.Interval(-300, 300), rg.Interval(-300, 300),
                rg.Interval(0, 300)).ToBrep()
    rib = one(rg.Brep.CreateBooleanIntersection([rib], [fa], TOL))
    rib = one(rg.Brep.CreateBooleanIntersection([rib], [fb], TOL))

    # foot: a horizontal plate under the B pad, 70 x 60 footprint, 6 thick, just in front
    # of the B face, so the part has a large flat bed face and prints upright
    fp = rg.Plane(rg.Point3d(OB.X, OB.Y, OB.Z - B_HALF_Y), plBu.XAxis, nB)
    foot = rg.Box(fp, rg.Interval(-FOOT_HALF_W, FOOT_HALF_W), rg.Interval(1.0, FOOT_DEPTH),
                  rg.Interval(0.0, FOOT_T)).ToBrep()

    parts = [padA, padB, rib, foot]

    # crevice filler at the A end of the rib (the B-end joint is already 48 degrees)
    ang_a = math.degrees(rg.Vector3d.VectorAngle(axis, nA))
    ang_b = math.degrees(rg.Vector3d.VectorAngle(-axis, nB))
    info.append("rib axis: angle to nA %.1f, to -nB %.1f, from horizontal %.1f, length %.1f"
                % (ang_a, ang_b, math.degrees(math.asin(max(-1.0, min(1.0, axis.Z)))), L))
    for s in (1, -1):
        g = gusset(PA, axis, nA, A_THICK, PAD_DEPTH, s, t, info, "A")
        if g is not None:
            parts.append(g)

    body = one(rg.Brep.CreateBooleanUnion(parts, TOL))
    body.MergeCoplanarFaces(0.01)

    # clearance holes along every bolt axis
    cutters = []
    for b in RES["bolts"]:
        ax = vec(b["axis"])
        pos = pt(b["position"])
        d = HOLE_DIA[b["size"]]
        start = pos - ax * 1.0
        cutters.append(rg.Cylinder(rg.Circle(rg.Plane(start, ax), d / 2), 10.0).ToBrep(True, True))
    body = one(rg.Brep.CreateBooleanDifference([body], cutters, TOL))
    body.MergeCoplanarFaces(0.01)
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
            pl = rg.Plane(pos + ax * h, ax)
            r = 2.0
            while r <= 72.0:
                deg = 0
                while deg < 360:
                    a = math.radians(deg)
                    p = pl.Origin + pl.XAxis * (r * math.cos(a)) + pl.YAxis * (r * math.sin(a))
                    if body.IsPointInside(p, TOL, False):
                        blocked[deg] = True
                    deg += 4
                r += 2.5
            h += 4.0
        free = []
        for s in range(360):
            if all(not blocked[(s + k) % 360] for k in range(0, 61, 2)):
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
        body = build(report)
        if not body.IsSolid or not body.IsValid:
            raise RuntimeError("body is not a valid closed solid")
        report.append("volume %.1f mm3" % body.GetVolume())
        bb = body.GetBoundingBox(True)
        report.append("bbox min %s max %s" % (bb.Min, bb.Max))
        for e in body.Edges:
            fs = e.AdjacentFaces()
            if len(fs) != 2:
                continue
            mid = e.PointAtNormalizedLength(0.5)
            ns = []
            for fi in fs:
                f = body.Faces[fi]
                ok, u, v = f.ClosestPoint(mid)
                ns.append(f.NormalAt(u, v))
            ang = math.degrees(rg.Vector3d.VectorAngle(ns[0], ns[1]))
            if ang < 60.0 or ang > 120.0:
                report.append("dbg edge %d angle %.1f mid %.1f %.1f %.1f len %.1f" % (
                    e.EdgeIndex, ang, mid.X, mid.Y, mid.Z, e.GetLength()))
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
