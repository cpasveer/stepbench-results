#! python3
"""Connecting bracket, designed to be printed standing on the pipe-axis direction (everything is a vertical prism).

Parts: two conformal pads on the pipe (bars along the pipe axis), a conformal strip tying them, a wall
(beam) leaving the pipe towards mount B, a foot block beside B's lower edge, and a 4-bolt frame plate on B.
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
BOLTS = {b["id"]: b for b in POSE["resolved"]["bolts"]}
PA = POSE["placements"]["A"]
PB = POSE["placements"]["B"]
TOL = 0.001

# ---------------- parameters (local pipe frame, x along pipe axis = print "up") ----------------
R_PIPE = 34.0
PAD_T = 4.0
XB = -23.5             # bed level (x)
PAD_XT = 23.5
PAD_HW = 6.5
STRIP_XT = -10.0
BR_RO, BR_RI = 36.5, 33.0
PL_T = 7.0
PL_H = 17.5
WIN = 8.0
FOOT_U = (-32.0, -16.5)
FOOT_V = (-17.5, 17.5)
FOOT_XT = -12.0
WALL_W = 12.0
WALL_Q0 = (-4.0, 30.0)
WALL_XT0 = -8.0
WALL_XT1 = -12.0
NMIN = 1.0             # stand-off of free material from B's face


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


O = V(PA["origin"])
RM = PA["rotation"]
XA = rg.Vector3d(RM[0][0], RM[1][0], RM[2][0])
YA = rg.Vector3d(RM[0][1], RM[1][1], RM[2][1])
ZA = rg.Vector3d(RM[0][2], RM[1][2], RM[2][2])


def to_local_pt(p):
    d = rg.Vector3d(p[0] - O.X, p[1] - O.Y, p[2] - O.Z)
    return rg.Point3d(d * XA, d * YA, d * ZA)


def to_local_vec(v):
    w = V(v)
    return rg.Vector3d(w * XA, w * YA, w * ZA)


RB = PB["rotation"]
BO = to_local_pt(PB["origin"])
UB = to_local_vec([RB[0][0], RB[1][0], RB[2][0]])
VB = to_local_vec([RB[0][1], RB[1][1], RB[2][1]])
NB = to_local_vec([RB[0][2], RB[1][2], RB[2][2]])

EX = rg.Vector3d(1, 0, 0)
A_ROW = {1: to_local_vec(BOLTS["A1"]["axis"]), 2: to_local_vec(BOLTS["A3"]["axis"])}
for _a in A_ROW.values():
    _a.Unitize()


def bpt(u, v, n):
    return BO + UB * u + VB * v + NB * n


def box(origin, xa, ya, xi, yi, zi):
    pl = rg.Plane(origin, xa, ya)
    return rg.Box(pl, rg.Interval(*xi), rg.Interval(*yi), rg.Interval(*zi)).ToBrep()


def fix(b):
    if b is None:
        raise RuntimeError("null brep")
    if not b.IsSolid:
        b = b.CapPlanarHoles(TOL)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def prism_yz(pts_yz, x0, x1):
    """vertical prism (along x) over a polygon given in (y, z)."""
    pts = [rg.Point3d(x0, p[0], p[1]) for p in pts_yz]
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    srf = rg.Surface.CreateExtrusion(crv, rg.Vector3d(x1 - x0, 0, 0))
    return fix(srf.ToBrep())


def plate_poly(u0, u1, v0, v1):
    out = []
    for (u, v) in ((u0, v0), (u1, v0), (u1, v1), (u0, v1)):
        p = bpt(u, v, 0)
        out.append((p.Y, p.Z))
    return out


def inter(a, b):
    r = rg.Brep.CreateBooleanIntersection([a], [b], TOL)
    if not r or len(r) != 1:
        raise RuntimeError("intersection failed")
    return r[0]


def halfspace_n(nmin):
    return box(BO, UB, VB, (-150, 150), (-150, 150), (nmin, 150))


def pad(row):
    a = A_ROW[row]
    org = rg.Point3d(0, 0, 0) + a * R_PIPE
    ya = rg.Vector3d.CrossProduct(a, EX)
    return box(org, EX, ya, (XB, PAD_XT), (-PAD_HW, PAD_HW), (-8.0, PAD_T))


def strip():
    def pt(r, ang):
        al = math.radians(ang)
        return rg.Point3d(XB, r * math.sin(al), r * math.cos(al))
    a0, a1 = -35.0, 35.0
    pc = rg.PolyCurve()
    pc.Append(rg.Arc(pt(BR_RO, a0), pt(BR_RO, 0), pt(BR_RO, a1)))
    pc.Append(rg.Line(pt(BR_RO, a1), pt(BR_RI, a1)))
    pc.Append(rg.Arc(pt(BR_RI, a1), pt(BR_RI, 0), pt(BR_RI, a0)))
    pc.Append(rg.Line(pt(BR_RI, a0), pt(BR_RO, a0)))
    srf = rg.Surface.CreateExtrusion(pc, rg.Vector3d(STRIP_XT - XB, 0, 0))
    return fix(srf.ToBrep())


def wall():
    q0 = rg.Point3d(0, WALL_Q0[0], WALL_Q0[1])
    ft = bpt(-25.0, 0.0, 0.0)
    q1 = rg.Point3d(0, ft.Y, ft.Z)
    d = q1 - q0
    length = d.Length
    d.Unitize()
    lat = rg.Vector3d(0, -d.Z, d.Y)

    def top(s):
        return WALL_XT0 + (WALL_XT1 - WALL_XT0) * s / length

    s0, s1 = -8.0, length + 2.0
    pts = []
    for (s, x) in ((s0, XB), (s1, XB), (s1, top(s1)), (s0, top(s0))):
        pts.append(q0 + d * s + EX * x)
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    crv.Translate(lat * (-WALL_W / 2))
    srf = rg.Surface.CreateExtrusion(crv, lat * WALL_W)
    return fix(srf.ToBrep())


def plate():
    outer = inter(prism_yz(plate_poly(-PL_H, PL_H, -PL_H, PL_H), -80, 80),
                  box(BO, UB, VB, (-150, 150), (-150, 150), (0, PL_T)))
    win = prism_yz(plate_poly(-WIN, WIN, -WIN, WIN), -80, 80)
    r = rg.Brep.CreateBooleanDifference([outer], [win], TOL)
    if not r or len(r) != 1:
        raise RuntimeError("window failed")
    return r[0]


def foot():
    p = prism_yz(plate_poly(FOOT_U[0], FOOT_U[1], FOOT_V[0], FOOT_V[1]), XB, FOOT_XT)
    return inter(p, halfspace_n(NMIN))


def cyl(base, axis, radius, height):
    ax = rg.Vector3d(axis)
    ax.Unitize()
    c = rg.Circle(rg.Plane(base, ax), radius)
    return rg.Cylinder(c, height).ToBrep(True, True)


def holes():
    hs = []
    for k, b in BOLTS.items():
        d = 5.5 if b["size"] == "M5" else 4.5
        p = to_local_pt(b["position"])
        a = to_local_vec(b["axis"])
        a.Unitize()
        hs.append(cyl(p - a * 8.0, a, d / 2, 8.0 + (9.0 if b["size"] == "M5" else 6.0 + (PL_T - 3.5))))
    return hs


def build():
    wl = inter(wall(), halfspace_n(NMIN))
    pieces = [pad(1), pad(2), strip(), wl, plate(), foot()]
    for i, p in enumerate(pieces):
        if p is None or not p.IsSolid:
            raise RuntimeError("piece %d not solid" % i)
    r = rg.Brep.CreateBooleanUnion(pieces, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("union failed (%s)" % (len(r) if r else 0))
    body = r[0]
    pipe = cyl(rg.Point3d(-200, 0, 0), EX, R_PIPE, 400.0)
    r = rg.Brep.CreateBooleanDifference([body], [pipe], TOL)
    if not r or len(r) != 1:
        raise RuntimeError("pipe cut failed")
    body = r[0]
    r = rg.Brep.CreateBooleanDifference([body], holes(), TOL)
    if not r or len(r) != 1:
        raise RuntimeError("hole cut failed")
    body = r[0]
    body.MergeCoplanarFaces(TOL)
    xf = rg.Transform.Identity
    cols = [XA, YA, ZA]
    for j in range(3):
        xf[0, j] = cols[j].X
        xf[1, j] = cols[j].Y
        xf[2, j] = cols[j].Z
    xf[0, 3] = O.X
    xf[1, 3] = O.Y
    xf[2, 3] = O.Z
    body.Transform(xf)
    return body


def overhang_report(b, up, log, limit=45.0):
    bad = {}
    tot = 0.0
    for i, f in enumerate(b.Faces):
        fb = f.ToBrep()
        area = rg.AreaMassProperties.Compute(fb).Area
        dom_u = f.Domain(0)
        dom_v = f.Domain(1)
        n_ok = 0
        n_bad = 0
        worst = 0.0
        N = 7
        for iu in range(N):
            for iv in range(N):
                u = dom_u.ParameterAt((iu + 0.5) / N)
                v = dom_v.ParameterAt((iv + 0.5) / N)
                if f.IsPointOnFace(u, v) == rg.PointFaceRelation.Exterior:
                    continue
                nv = f.NormalAt(u, v)
                if f.OrientationIsReversed:
                    pass
                s = -(nv * up)  # 1 for a face looking straight down
                if s > 0.9995:
                    n_ok += 1
                    continue
                ang = math.degrees(math.asin(max(-1.0, min(1.0, s))))
                if ang > limit:
                    n_bad += 1
                    worst = max(worst, ang)
                else:
                    n_ok += 1
        if n_bad:
            frac = n_bad / float(n_bad + n_ok)
            bad[i] = (round(area * frac, 1), round(worst, 1))
            tot += area * frac
    log.append("overhang faces (area, worst deg): %s total=%.1f" % (bad, tot))


def same_surface_pairs(b, log):
    res = []
    for i, f in enumerate(b.Faces):
        for j in f.AdjacentFaces():
            if j <= i:
                continue
            g = b.Faces[j]
            ok1, p1 = f.TryGetPlane(0.01)
            ok2, p2 = g.TryGetPlane(0.01)
            if ok1 and ok2:
                if abs(abs(p1.Normal * p2.Normal) - 1) < 1e-4 and abs(p1.DistanceTo(p2.Origin)) < 0.01:
                    res.append((i, j, "plane"))
                continue
            c1, y1 = f.TryGetCylinder(0.01)
            c2, y2 = g.TryGetCylinder(0.01)
            if c1 and c2:
                if abs(y1.Radius - y2.Radius) < 0.01 and abs(abs(y1.Axis * y2.Axis) - 1) < 1e-4:
                    d = rg.Vector3d(y2.BasePlane.Origin - y1.BasePlane.Origin)
                    cr = rg.Vector3d.CrossProduct(d, y1.Axis)
                    if cr.Length < 0.01:
                        res.append((i, j, "cyl"))
    log.append("same-surface adjacent pairs: %s" % res)


def qa(b, log):
    same_surface_pairs(b, log)
    log.append("solid=%s valid=%s faces=%d edges=%d vol=%.1f" % (b.IsSolid, b.IsValid, b.Faces.Count, b.Edges.Count, rg.VolumeMassProperties.Compute(b).Volume))
    small = []
    for i, f in enumerate(b.Faces):
        a = rg.AreaMassProperties.Compute(f.ToBrep()).Area
        if a < 5:
            small.append((i, round(a, 2)))
    log.append("small faces: %s" % small)
    short = [round(e.GetLength(), 2) for e in b.Edges if e.GetLength() < 1.0]
    log.append("short edges: %s" % short)
    bb = b.GetBoundingBox(True)
    log.append("bbox %s %s" % (bb.Min, bb.Max))
    overhang_report(b, XA, log)


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    log = []
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        b = build()
        qa(b, log)
        doc.Objects.AddBrep(b)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(log) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
