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

# ---------------- parameters (local pipe frame, x along the pipe axis) ----------------
R_PIPE = 34.0
PAD_T = 4.0
XB_LOW = -40.0         # solids reach below the print bed; the bed plane cuts them
PAD_XT = {1: 22.5, 2: 23.5}
PAD_HW = 5.0
TOP_W = -3.0           # common top plane of strip and wall
BLOCK_BELOW = 0.0      # block top lies this far below the plate's front corner
BR_RO, BR_RI = 36.5, 33.0
PL_T = 11.0
PL_H = 17.5
WIN = 8.0
DRAFT = math.tan(math.radians(5.0))   # lean of the plate's lower edge wall (printing)
WALL_W = 24.0          # wall width at the print bed
W_TOP = 12.0           # wall width at its top
X_REF = -20.5          # nominal bed level for the wall taper
WALL_Q0 = (-4.0, 30.0)
WALL_U_END = -34.0
NMIN = 0.4             # stand-off of free material from B's face
BED_TILT = math.radians(2.0)   # print bed plane: through the plate's lowest edge, slightly tilted


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


def poly_face(pts):
    pl = rg.Polyline(list(pts) + [pts[0]])
    r = rg.Brep.CreatePlanarBreps(pl.ToNurbsCurve(), TOL)
    if not r:
        raise RuntimeError("planar face failed")
    return r[0]


def prism_pts(bot, top):
    n = len(bot)
    faces = [poly_face(bot), poly_face(top)]
    for i in range(n):
        j = (i + 1) % n
        faces.append(poly_face([bot[i], bot[j], top[j], top[i]]))
    j = rg.Brep.JoinBreps(faces, TOL)
    if not j or len(j) != 1:
        raise RuntimeError("prism join failed")
    return fix(j[0])


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
    return box(org, EX, ya, (XB_LOW, PAD_XT[row]), (-PAD_HW, PAD_HW), (-8.0, PAD_T))


def strip():
    def pt(r, ang):
        al = math.radians(ang)
        return rg.Point3d(XB_LOW, r * math.sin(al), r * math.cos(al))
    a0, a1 = -35.0, 35.0
    pc = rg.PolyCurve()
    pc.Append(rg.Arc(pt(BR_RO, a0), pt(BR_RO, 0), pt(BR_RO, a1)))
    pc.Append(rg.Line(pt(BR_RO, a1), pt(BR_RI, a1)))
    pc.Append(rg.Arc(pt(BR_RI, a1), pt(BR_RI, 0), pt(BR_RI, a0)))
    pc.Append(rg.Line(pt(BR_RI, a0), pt(BR_RO, a0)))
    srf = rg.Surface.CreateExtrusion(pc, rg.Vector3d(TOP_W - XB_LOW, 0, 0))
    return fix(srf.ToBrep())


def wall():
    q0, d, length, lat = wall_frame()
    s0 = -8.0
    wb = W_TOP + (WALL_W - W_TOP) * (TOP_W - XB_LOW) / (TOP_W - X_REF)   # width at the (low) bottom of the prism

    def ring(x, w):
        return [q0 + d * s0 - lat * (w / 2) + EX * x, q0 + d * length - lat * (w / 2) + EX * x,
                q0 + d * length + lat * (w / 2) + EX * x, q0 + d * s0 + lat * (w / 2) + EX * x]
    return prism_pts(ring(XB_LOW, wb), ring(TOP_W, W_TOP))


def plate():
    t = PL_T * DRAFT
    bot = [bpt(-PL_H, -PL_H, 0), bpt(PL_H, -PL_H, 0), bpt(PL_H, PL_H, 0), bpt(-PL_H, PL_H, 0)]
    top = [bpt(-PL_H + t, -PL_H, PL_T), bpt(PL_H, -PL_H, PL_T), bpt(PL_H, PL_H, PL_T), bpt(-PL_H + t, PL_H, PL_T)]
    outer = prism_pts(bot, top)
    # opening: open to the upper edge (u large), so no downward-facing wall appears in it
    n0, n1 = -1.0, PL_T + 1.0
    uo = 40.0
    wb = [bpt(-WIN, -WIN, n0), bpt(uo, -WIN, n0), bpt(uo, WIN, n0), bpt(-WIN, WIN, n0)]
    wt = [bpt(-WIN, -WIN, n1), bpt(uo, -WIN, n1), bpt(uo, WIN, n1), bpt(-WIN, WIN, n1)]
    win = prism_pts(wb, wt)
    cutters = [win]
    # slanted tips of the two prongs (different planes, so no two faces share a plane)
    tip = math.radians(10.0)
    for sg in (1.0, -1.0):
        p0 = (PL_H - 0.3, sg * WIN)
        nh = (math.cos(tip), sg * math.sin(tip))
        th = (-sg * math.sin(tip), math.cos(tip))
        def corner(a, b, p0=p0, nh=nh, th=th):
            return (p0[0] + th[0] * a + nh[0] * b, p0[1] + th[1] * a + nh[1] * b)
        pts = [corner(-30, 0), corner(30, 0), corner(30, 30), corner(-30, 30)]
        if sg < 0:
            pts = pts[::-1]
        cb = [bpt(u, v, n0) for (u, v) in pts]
        ct = [bpt(u, v, n1) for (u, v) in pts]
        cutters.append(prism_pts(cb, ct))
    r = rg.Brep.CreateBooleanDifference([outer], cutters, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("window failed")
    return r[0]


def wall_frame():
    q0 = rg.Point3d(0, WALL_Q0[0], WALL_Q0[1])
    ft = bpt(WALL_U_END, 0.0, 0.0)
    q1 = rg.Point3d(0, ft.Y, ft.Z)
    d = q1 - q0
    length = d.Length
    d.Unitize()
    lat = rg.Vector3d(0, -d.Z, d.Y)
    return q0, d, length, lat


def block():
    """foot block: low plate carried under the wall's end, flaring to the plate's width along its lower edge."""
    q0, d, length, lat = wall_frame()
    ctop = bpt(-PL_H + PL_T * DRAFT, 0.0, PL_T)
    xbt = ctop.X - BLOCK_BELOW
    w_at = W_TOP + (WALL_W - W_TOP) * (TOP_W - xbt) / (TOP_W - X_REF)   # wall width at the block's top level
    hw0 = w_at / 2 - 3.5
    hw1 = WALL_W / 2 + 0.8
    c1 = bpt(-PL_H, -PL_H, 0)
    c2 = bpt(-PL_H, PL_H, 0)
    c1 = rg.Point3d(0, c1.Y, c1.Z)
    c2 = rg.Point3d(0, c2.Y, c2.Z)
    if (c1 - q0) * lat > (c2 - q0) * lat:
        c1, c2 = c2, c1      # c1 on the -lat side
    a_l = q0 + d * (length - 3.0) - lat * hw0
    a_r = q0 + d * (length - 3.0) + lat * hw0
    b_l = q0 + d * length - lat * hw1
    b_r = q0 + d * length + lat * hw1
    poly = [a_l, b_l, c1, c2, b_r, a_r]
    bot = [rg.Point3d(XB_LOW, p.Y, p.Z) for p in poly]
    top = [rg.Point3d(xbt, p.Y, p.Z) for p in poly]
    return inter(prism_pts(bot, top), halfspace_n(NMIN))


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


def bed_geometry():
    e0 = bpt(-PL_H, 0.0, 0.0)           # a point of the plate's lowest edge
    wd = rg.Vector3d.CrossProduct(VB, EX)
    wd.Unitize()
    up = EX * math.cos(BED_TILT) - wd * math.sin(BED_TILT)
    up.Unitize()
    return e0, up


def bed_halfspace():
    e0, up = bed_geometry()
    # a plane with normal 'up' through the plate edge; keep everything above it
    ref = rg.Vector3d(0, 1, 0)
    xa = rg.Vector3d.CrossProduct(up, ref)
    xa.Unitize()
    ya = rg.Vector3d.CrossProduct(up, xa)
    ya.Unitize()
    pl = rg.Plane(e0, xa, ya)
    return rg.Box(pl, rg.Interval(-300, 300), rg.Interval(-300, 300), rg.Interval(0, 300)).ToBrep()


def build():
    low_pieces = [pad(1), pad(2), strip(), wall(), block()]
    for i, p in enumerate(low_pieces):
        if p is None or not p.IsSolid:
            raise RuntimeError("piece %d not solid" % i)
    r = rg.Brep.CreateBooleanUnion(low_pieces, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("low union failed (%s)" % (len(r) if r else 0))
    low = inter(r[0], bed_halfspace())
    pl = plate()
    r = rg.Brep.CreateBooleanUnion([low, pl], TOL)
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
    global LOCAL_BODY
    LOCAL_BODY = body.DuplicateBrep()
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
        fb = f.DuplicateFace(False)
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


def dump_faces(b, log):
    for i, f in enumerate(b.Faces):
        am = rg.AreaMassProperties.Compute(f.DuplicateFace(False))
        c = am.Centroid
        lc = to_local_pt((c.X, c.Y, c.Z))
        ok, pl = f.TryGetPlane(0.01)
        ln = pl.Normal if ok else None
        nl = to_local_vec((ln.X, ln.Y, ln.Z)) if ok else None
        log.append("  F%d %s area=%.1f c=(%.1f,%.1f,%.1f) n=%s adj=%s" % (i, "plane" if ok else "curved", am.Area, lc.X, lc.Y, lc.Z, ("(%.2f,%.2f,%.2f)" % (nl.X, nl.Y, nl.Z)) if ok else "", list(f.AdjacentFaces())))


def surface_groups(b, log):
    groups = {}
    for i, f in enumerate(b.Faces):
        ok, pl = f.TryGetPlane(0.01)
        if ok:
            n = pl.Normal
            if (n.X, n.Y, n.Z) < (0, 0, 0):
                n = -n
            d = pl.DistanceTo(rg.Point3d(0, 0, 0)) if False else (rg.Vector3d(pl.Origin) * n)
            key = ("P", round(n.X, 3), round(n.Y, 3), round(n.Z, 3), round(d, 2))
        else:
            c, cy = f.TryGetCylinder(0.01)
            if c:
                a = cy.Axis
                o = rg.Vector3d(cy.BasePlane.Origin)
                perp = o - a * (o * a)
                key = ("C", round(abs(a.X), 3), round(abs(a.Y), 3), round(abs(a.Z), 3), round(cy.Radius, 2), round(perp.X, 1), round(perp.Y, 1), round(perp.Z, 1))
            else:
                continue
        groups.setdefault(key, []).append(i)
    multi = dict((k, v) for k, v in groups.items() if len(v) > 1)
    log.append("surface groups with >1 face: %s" % multi)
    for k, v in multi.items():
        for i in v:
            fb = b.Faces[i].DuplicateFace(False)
            am = rg.AreaMassProperties.Compute(fb)
            cen = am.Centroid
            lc = to_local_pt((cen.X, cen.Y, cen.Z))
            bb = fb.GetBoundingBox(True)
            l0 = to_local_pt((bb.Min.X, bb.Min.Y, bb.Min.Z))
            l1 = to_local_pt((bb.Max.X, bb.Max.Y, bb.Max.Z))
            verts = []
            for v in fb.Vertices:
                q = to_local_pt((v.Location.X, v.Location.Y, v.Location.Z))
                verts.append("(%.1f,%.1f,%.1f)" % (q.X, q.Y, q.Z))
            log.append("  group %s face %d area %.1f centroid local (%.1f, %.1f, %.1f) verts %s" % (k[:2], i, am.Area, lc.X, lc.Y, lc.Z, " ".join(verts)))


def edge_angles(b, log, limit=75.0):
    out = []
    for i, e in enumerate(b.Edges):
        af = list(e.AdjacentFaces())
        if len(af) != 2:
            continue
        crv = e.EdgeCurve
        t = crv.Domain.ParameterAt(0.5)
        P = crv.PointAt(t)
        T = crv.TangentAt(t)
        f1 = b.Faces[af[0]]
        f2 = b.Faces[af[1]]
        ok1, u1, v1 = f1.ClosestPoint(P)
        ok2, u2, v2 = f2.ClosestPoint(P)
        if not (ok1 and ok2):
            continue
        n1 = f1.NormalAt(u1, v1)
        n2 = f2.NormalAt(u2, v2)
        if f1.OrientationIsReversed:
            n1 = -n1
        if f2.OrientationIsReversed:
            n2 = -n2
        ang = math.degrees(math.acos(max(-1.0, min(1.0, n1 * n2))))
        wedge = 180.0 - ang
        m = rg.Vector3d.CrossProduct(T, n1)
        m.Unitize()
        pos = P + m * 0.05
        cp = f1.ClosestPoint(pos)
        d_plus = pos.DistanceTo(f1.PointAt(cp[1], cp[2])) if cp[0] else 9
        pos2 = P - m * 0.05
        cp2 = f1.ClosestPoint(pos2)
        d_minus = pos2.DistanceTo(f1.PointAt(cp2[1], cp2[2])) if cp2[0] else 9
        b1 = m if d_plus < d_minus else -m
        concave = (b1 * n2) > 0
        if wedge < 50.0:
            l1 = to_local_vec((n1.X, n1.Y, n1.Z))
            l2 = to_local_vec((n2.X, n2.Y, n2.Z))
            a0 = to_local_pt((crv.PointAtStart.X, crv.PointAtStart.Y, crv.PointAtStart.Z))
            a1 = to_local_pt((crv.PointAtEnd.X, crv.PointAtEnd.Y, crv.PointAtEnd.Z))
            log.append("edge %d faces %d,%d n1=(%.2f,%.2f,%.2f) n2=(%.2f,%.2f,%.2f) from (%.1f,%.1f,%.1f) to (%.1f,%.1f,%.1f)" % (i, af[0], af[1], l1.X, l1.Y, l1.Z, l2.X, l2.Y, l2.Z, a0.X, a0.Y, a0.Z, a1.X, a1.Y, a1.Z))
        if wedge < limit:
            out.append((i, "concave" if concave else "convex", round(wedge, 1), round(e.GetLength(), 1), (round(P.X, 1), round(P.Y, 1), round(P.Z, 1))))
    log.append("sharp edges (idx, type, wedge deg, length, mid): %s" % out)


def qa(b, log):
    edge_angles(b, log)
    dump_faces(b, log)
    surface_groups(b, log)
    same_surface_pairs(b, log)
    log.append("solid=%s valid=%s faces=%d edges=%d vol=%.1f" % (b.IsSolid, b.IsValid, b.Faces.Count, b.Edges.Count, rg.VolumeMassProperties.Compute(b).Volume))
    small = []
    for i, f in enumerate(b.Faces):
        a = rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area
        if a < 5:
            small.append((i, round(a, 2)))
    log.append("small faces: %s" % small)
    short = [round(e.GetLength(), 2) for e in b.Edges if e.GetLength() < 1.0]
    log.append("short edges: %s" % short)
    bb = b.GetBoundingBox(True)
    log.append("bbox %s %s" % (bb.Min, bb.Max))
    e0, upl = bed_geometry()
    upw = XA * upl.X + YA * upl.Y + ZA * upl.Z
    log.append("bed up (world): %.3f %.3f %.3f" % (upw.X, upw.Y, upw.Z))
    overhang_report(b, upw, log)


LOCAL_BODY = None


def dump_mesh(brep, path):
    mp = rg.MeshingParameters.QualityRenderMesh
    meshes = rg.Mesh.CreateFromBrep(brep, mp)
    vs = []
    fs = []
    off = 0
    for m in meshes:
        m.Faces.ConvertQuadsToTriangles()
        for v in m.Vertices:
            vs.append([round(v.X, 4), round(v.Y, 4), round(v.Z, 4)])
        for f in m.Faces:
            fs.append([f.A + off, f.B + off, f.C + off])
        off += m.Vertices.Count
    json.dump({"v": vs, "f": fs}, open(path, "w"))


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    log = []
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        b = build()
        qa(b, log)
        try:
            dump_mesh(LOCAL_BODY, os.path.join(HERE, "scratch", "mesh_local.json"))
        except Exception as ex:
            log.append("mesh dump failed: %s" % ex)
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
