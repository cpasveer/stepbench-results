#! python3
"""Bracket joining flange A and block B. Built to print upside down (top face on the bed)."""
import json, os, math, traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = {b["id"]: b for b in POSE["bolts"]}
TOL = 0.001

Z_TOP = 37.5          # bed plane (print upside down)
T_A = 3.8
T_B = 4.55
HOLE_D = {"M5": 5.5, "M6": 6.7}
RELIEF_Z = 28.8       # above this the B plate is set back 1 mm from B's face
DBG = []


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def frame(face):
    o = rg.Point3d(*face["origin"])
    n = V(face["normal"]); n.Unitize()
    z = rg.Vector3d(0, 0, 1)
    t = rg.Vector3d(-n.Y, n.X, 0)   # t x z = n
    return o, t, n


def solid_from_polygon(pts, vec):
    pts = list(pts) + [pts[0]]
    crv = rg.PolylineCurve(pts)
    srf = rg.Surface.CreateExtrusion(crv, vec)
    b = srf.ToBrep().CapPlanarHoles(TOL)
    if b is None or not b.IsSolid:
        raise RuntimeError("extrusion not solid")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def union(breps):
    res = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("union failed: %s" % (None if not res else len(res)))
    return res[0]


def diff(a, bs):
    res = rg.Brep.CreateBooleanDifference([a], bs, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("difference failed: %s" % (None if not res else len(res)))
    return res[0]


def build():
    Ao, tA, nA = frame(FACES["A"])
    Bo, tB, nB = frame(FACES["B"])
    zc = rg.Vector3d(0, 0, 1)

    def gA(u, n, z):   # A frame (tA, nA) + absolute z
        return rg.Point3d(Ao.X + u * tA.X + n * nA.X, Ao.Y + u * tA.Y + n * nA.Y, z)

    def gB(u, n, z):
        return rg.Point3d(Bo.X + u * tB.X + n * nB.X, Bo.Y + u * tB.Y + n * nB.Y, z)

    zA = Ao.Z
    # ---------------- plate A (extruded along nA from the face)
    polyA = [(-7.73, 44), (7.73, 44), (35.5, 6), (35.5, -6), (26, -25), (15, -32.5),
             (-15, -32.5), (-26, -25), (-35.5, -6), (-35.5, 6)]
    ptsA = [gA(u, 0.0, zA + v) for (u, v) in polyA]
    plateA = solid_from_polygon(ptsA, nA * T_A)

    # ---------------- plate B
    ztop = Z_TOP + 10.0
    polyB = [(-21.5, ztop), (21.5, ztop), (21.5, 14), (9.5, 14), (9.5, -35.5), (21.5, -53.5), (21.5, -64.5),
             (-21.5, -64.5), (-21.5, -53.5), (-9.5, -35.5), (-9.5, 14), (-21.5, 14)]
    ptsB = [gB(u, 0.0, z) for (u, z) in polyB]
    plateB = solid_from_polygon(ptsB, nB * T_B)

    # ---------------- fin: plan polygon, full height, cut by a sloping plane underneath
    fin_plan = [gA(-4.5, 2.0, 0), gA(9.7, 2.0, 0), gA(9.7, 39.15, 0), gB(-7.0, 27.0, 0),
                gB(-7.0, 2.3, 0), gB(7.0, 2.3, 0), gB(8.8, 41.6, 0), gA(-4.5, 32.6, 0)]
    fin_plan = [rg.Point3d(p.X, p.Y, -70.0) for p in fin_plan]
    fin = solid_from_polygon(fin_plan, rg.Vector3d(0, 0, Z_TOP + 80.0))
    s0 = gA(0, 2.0, 0); s1 = gB(0, 2.3, 0)
    g = rg.Vector3d(s1.X - s0.X, s1.Y - s0.Y, 0); L = g.Length; g.Unitize()
    nrm = rg.Vector3d(g.Y, -g.X, 0)
    zs0, zs1 = 12.0, -45.0
    def zline(s):
        return zs0 + (zs1 - zs0) * s / L
    cut_pts = []
    for (s, z) in [(-15, zline(-15)), (L + 15, zline(L + 15)), (L + 15, -140), (-15, -140)]:
        cut_pts.append(rg.Point3d(s0.X + g.X * s - nrm.X * 40, s0.Y + g.Y * s - nrm.Y * 40, z))
    cutter = solid_from_polygon(cut_pts, nrm * 80)
    fin = diff(fin, [cutter])

    body = union([plateA, plateB, fin])
    # flat top at the bed: cut everything above Z_TOP
    top = rg.Box(rg.Plane(rg.Point3d(Bo.X, Bo.Y, Z_TOP), rg.Vector3d.XAxis, rg.Vector3d.YAxis),
                 rg.Interval(-200, 200), rg.Interval(-200, 200), rg.Interval(0, 40)).ToBrep()
    body = diff(body, [top])

    # ---------------- relief on plate B above RELIEF_Z (45-ish chamfer, 1 mm set-back)
    rel = []
    for (n, z) in [(-1.0, RELIEF_Z), (0.0, RELIEF_Z), (1.0, RELIEF_Z + 2.0), (1.0, 55.0), (-1.0, 55.0)]:
        rel.append(gB(-30.0, n, z))
    # extrusion along tB (60 long)
    relief = solid_from_polygon(rel, tB * 60.0)
    body = diff(body, [relief])

    # ---------------- holes
    cutters = []
    for bid, b in BOLTS.items():
        pos = rg.Point3d(*b["position"]); ax = V(b["axis"]); ax.Unitize()
        r = HOLE_D[b["size"]] / 2.0
        base = pos - ax * 1.0
        cyl = rg.Cylinder(rg.Circle(rg.Plane(base, ax), r), 12.0).ToBrep(True, True)
        cutters.append(cyl)
    body = diff(body, cutters)
    body.MergeCoplanarFaces(TOL)

    # ---------------- local checks
    # tool / head columns
    tot = 0.0
    for bid, b in BOLTS.items():
        pos = rg.Point3d(*b["position"]); ax = V(b["axis"]); ax.Unitize()
        isA = bid.startswith("A")
        T = T_A if isA else T_B
        r = 5.0 if isA else 5.75
        base = pos + ax * T
        cyl = rg.Cylinder(rg.Circle(rg.Plane(base, ax), r), (35.0 if isA else 36.0)).ToBrep(True, True)
        res = rg.Brep.CreateBooleanIntersection([body], [cyl], TOL)
        v = 0.0
        if res:
            for x in res:
                v += abs(x.GetVolume())
        tot += v
        if v > 0.001:
            DBG.append("column clash %s vol %.2f" % (bid, v))
    DBG.append("column clash total %.3f" % tot)
    # overhang: area of faces whose normal points up more than 45 deg from vertical, not at bed
    mp = rg.MeshingParameters.Default
    meshes = rg.Mesh.CreateFromBrep(body, mp)
    bad = 0.0
    badpts = {}
    for m in meshes:
        m.FaceNormals.ComputeFaceNormals()
        for i in range(m.Faces.Count):
            f = m.Faces[i]
            pa, pb, pc = m.Vertices[f.A], m.Vertices[f.B], m.Vertices[f.C]
            ar = 0.5 * rg.Vector3d.CrossProduct(rg.Point3d(pb) - rg.Point3d(pa), rg.Point3d(pc) - rg.Point3d(pa)).Length
            nz = m.FaceNormals[i].Z
            zc_ = (pa.Z + pb.Z + pc.Z) / 3.0
            if nz > 0.7071 + 1e-6 and zc_ < Z_TOP - 0.05:
                bad += ar
                cx = (pa.X + pb.X + pc.X) / 3.0; cy = (pa.Y + pb.Y + pc.Y) / 3.0
                key = (round(cx / 5) * 5, round(cy / 5) * 5, round(zc_ / 5) * 5)
                badpts[key] = badpts.get(key, 0) + ar
    DBG.append("print-down overhang area (flipped) %.2f" % bad)
    for k in sorted(badpts, key=lambda k: -badpts[k])[:8]:
        DBG.append("  overhang near %s area %.1f" % (str(k), badpts[k]))
    # topology sanity: coplanar planar face groups, small creases
    planes = []
    for i, f in enumerate(body.Faces):
        ok, pl = f.TryGetPlane(0.001)
        if ok:
            planes.append((i, pl))
    for a in range(len(planes)):
        for b in range(a + 1, len(planes)):
            pa_, pb_ = planes[a][1], planes[b][1]
            if abs(rg.Vector3d.Multiply(pa_.Normal, pb_.Normal)) > 0.9999 and abs(pa_.DistanceTo(pb_.Origin)) < 0.01:
                DBG.append("coplanar faces %d %d" % (planes[a][0], planes[b][0]))
    for ei in range(body.Edges.Count):
        e = body.Edges[ei]
        fi = e.AdjacentFaces()
        if len(fi) == 2:
            t = e.Domain.Mid
            pt = e.PointAt(t)
            n = []
            for k in fi:
                f = body.Faces[k]
                ok, u, v = f.ClosestPoint(pt)
                nn = f.NormalAt(u, v)
                if f.OrientationIsReversed:
                    nn.Reverse()
                n.append(nn)
            ang = math.degrees(math.acos(max(-1, min(1, rg.Vector3d.Multiply(n[0], n[1])))))
            if ang < 10 and e.GetLength() > 0.01:
                DBG.append("small crease %.2f deg len %.2f faces %s at %s" % (ang, e.GetLength(), list(fi), e.PointAt(t)))
    DBG.append("volume %.1f faces %d valid %s solid %s" % (body.GetVolume(), body.Faces.Count, body.IsValid, body.IsSolid))
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(DBG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(DBG))
    if os.path.exists(OUT):
        os.remove(OUT)
