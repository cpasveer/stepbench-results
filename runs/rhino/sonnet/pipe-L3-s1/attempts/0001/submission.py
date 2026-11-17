#! python3
"""Connecting bracket: two conformal saddle pads on the pipe (bridged), one strut, one 4-bolt frame plate on B."""
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

# ---------------- parameters ----------------
R_PIPE = 34.0
PAD_T = 4.0            # grip on pipe
PAD_X = 23.5           # pad half length along pipe
PAD_HW1 = 6.5
PAD_HW2 = 8.5
BR_X0, BR_X1 = -10.0, 2.0
BR_RO, BR_RI = 36.5, 33.0
PL_T = 3.5             # plate thickness (grip on B)
PL_H = 17.5            # plate half size
WIN = 8.0              # window half size
TAB_U = (-34.0, -17.5)
TAB_V = (-26.0, -5.0)
TAB_N0 = 1.0
ARM_W = 11.0
ARM_XS = 1.0
ARM_T_UV = (-25.0, -15.0)
ARM_ABOT = 36.5
ARM_NTOP = 1.5


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


# local pipe frame: x along pipe axis; world = O + R * local
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


# B frame in local coordinates
RB = PB["rotation"]
BO = to_local_pt(PB["origin"])
UB = to_local_vec([RB[0][0], RB[1][0], RB[2][0]])
VB = to_local_vec([RB[0][1], RB[1][1], RB[2][1]])
NB = to_local_vec([RB[0][2], RB[1][2], RB[2][2]])


def bpt(u, v, n):
    return BO + UB * u + VB * v + NB * n


EX = rg.Vector3d(1, 0, 0)
A_ROW = {1: to_local_vec(BOLTS["A1"]["axis"]), 2: to_local_vec(BOLTS["A3"]["axis"])}


def box(origin, xa, ya, xi, yi, zi):
    pl = rg.Plane(origin, xa, ya)
    b = rg.Box(pl, rg.Interval(*xi), rg.Interval(*yi), rg.Interval(*zi))
    return b.ToBrep()


def pad(row, hw):
    a = A_ROW[row]
    a.Unitize()
    t = rg.Vector3d.CrossProduct(a, EX)
    t.Unitize()
    # plane origin on pipe surface along a, x=0
    org = rg.Point3d(0, 0, 0) + a * R_PIPE
    xa = EX
    ya = rg.Vector3d.CrossProduct(a, EX)  # x cross y = a
    return box(org, xa, ya, (-PAD_X, PAD_X), (-hw, hw), (-8.0, PAD_T))


def bridge():
    def pt(r, ang):
        al = math.radians(ang)
        return rg.Point3d(0, r * math.sin(al), r * math.cos(al))
    a0, a1 = -35.0, 35.0
    arc_o = rg.Arc(pt(BR_RO, a0), pt(BR_RO, 0), pt(BR_RO, a1))
    arc_i = rg.Arc(pt(BR_RI, a1), pt(BR_RI, 0), pt(BR_RI, a0))
    pc = rg.PolyCurve()
    pc.Append(arc_o)
    pc.Append(rg.Line(pt(BR_RO, a1), pt(BR_RI, a1)))
    pc.Append(arc_i)
    pc.Append(rg.Line(pt(BR_RI, a0), pt(BR_RO, a0)))
    if not pc.IsClosed:
        pc.MakeClosed(0.01)
    ext = rg.Extrusion.Create(pc, BR_X1 - BR_X0, True)
    br = ext.ToBrep()
    bb = br.GetBoundingBox(True)
    br.Translate(BR_X0 - bb.Min.X, 0, 0)
    return br


def quad(p0, p1, p2, p3):
    return rg.Brep.CreateFromCornerPoints(p0, p1, p2, p3, TOL)


def arm():
    a2 = A_ROW[2]
    a2.Unitize()
    tg = rg.Vector3d.CrossProduct(a2, EX)
    tg.Unitize()
    base = rg.Point3d(ARM_XS, 0, 0) + a2 * ARM_ABOT
    tm = bpt(ARM_T_UV[0], ARM_T_UV[1], 2.25)
    D = tm - base
    D.Unitize()
    low = []
    up = []
    for sx, st in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        p = base + EX * (sx * ARM_W / 2) + tg * (st * ARM_W / 2)
        low.append(p)
        rel = p - BO
        t = (ARM_NTOP - (rel * NB)) / (D * NB)
        up.append(p + D * t)
    faces = [quad(low[0], low[1], low[2], low[3]), quad(up[3], up[2], up[1], up[0])]
    for i in range(4):
        j = (i + 1) % 4
        faces.append(quad(low[i], low[j], up[j], up[i]))
    joined = rg.Brep.JoinBreps(faces, TOL)
    b = joined[0]
    b.Faces.SplitKinkyFaces() if False else None
    if not b.IsSolid:
        b = b.CapPlanarHoles(TOL) or b
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def plate():
    pl_org = BO
    outer = box(pl_org, UB, VB, (-PL_H, PL_H), (-PL_H, PL_H), (0, PL_T))
    win = box(pl_org, UB, VB, (-WIN, WIN), (-WIN, WIN), (-1, PL_T + 1))
    r = rg.Brep.CreateBooleanDifference([outer], [win], TOL)
    return r[0]


def tab():
    return box(BO, UB, VB, TAB_U, TAB_V, (TAB_N0, PL_T))


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
        hs.append(cyl(p - a * 8.0, a, d / 2, 8.0 + 9.0 if b["size"] == "M5" else 8.0 + 6.0))
    return hs


def union(breps):
    r = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not r:
        raise RuntimeError("union failed")
    return r


def build():
    pieces = [pad(1, PAD_HW1), pad(2, PAD_HW2), bridge(), arm(), plate(), tab()]
    for i, p in enumerate(pieces):
        if p is None or not p.IsSolid:
            raise RuntimeError("piece %d not solid" % i)
    body = union(pieces)
    if len(body) != 1:
        raise RuntimeError("union gave %d" % len(body))
    body = body[0]
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
    # to world
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


def qa(b, log):
    log.append("solid=%s valid=%s faces=%d edges=%d vol=%.1f" % (b.IsSolid, b.IsValid, b.Faces.Count, b.Edges.Count, rg.VolumeMassProperties.Compute(b).Volume))
    small = []
    for i, f in enumerate(b.Faces):
        a = rg.AreaMassProperties.Compute(f.ToBrep()).Area
        if a < 5:
            small.append((i, round(a, 2)))
    log.append("small faces: %s" % small)
    short = [round(e.GetLength(), 2) for e in b.Edges if e.GetLength() < 1.0]
    log.append("short edges: %s" % short)


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
