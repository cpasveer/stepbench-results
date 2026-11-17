#! python3
"""pipe-L2-s1: saddle clamp on the pipe, square flange on mount B, joined by a neck.

Built in Rhino (RhinoCommon) in a headless document, exported as output.step.
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
BOLTS = {b["id"]: b for b in RES["bolts"]}
TOL = 0.001
MSG = []


def log(*a):
    MSG.append(" ".join(str(x) for x in a))
    try:
        open(os.path.join(HERE, "scratch", "progress.txt"), "a").write(MSG[-1] + "\n")
    except Exception:
        pass


def V(v):
    return rg.Vector3d(v[0], v[1], v[2])


def P(v):
    return rg.Point3d(v[0], v[1], v[2])


# ---- pipe frame: X along axis, Y/Z across (bolts at +-35 deg from +Z)
RA = POSE["placements"]["A"]["rotation"]
OA = P(POSE["placements"]["A"]["origin"])
AX = rg.Vector3d(RA[0][0], RA[1][0], RA[2][0])
AY = rg.Vector3d(RA[0][1], RA[1][1], RA[2][1])
AZ = rg.Vector3d(RA[0][2], RA[1][2], RA[2][2])


def LA(x, y, z):
    return OA + AX * x + AY * y + AZ * z


# ---- B frame: U, W in the face, N outward normal
FB = RES["mounting_faces"][0]
OB = P(FB["origin"])
BN = V(FB["normal"])
BN.Unitize()
BW = rg.Vector3d(0, 0, 1)  # world Z lies in the face
BU = rg.Vector3d.CrossProduct(BW, BN)  # so that (U, W, N) is right-handed
BU.Unitize()


def LB(u, w, n):
    return OB + BU * u + BW * w + BN * n


# ---- dimensions
R_PIPE = 34.0
T_SAD = 7.0  # saddle thickness at the bolts (grip)
R_OUT = R_PIPE + T_SAD
PSI = math.radians(55.0)  # half angle of the seat arc
PHI = math.radians(35.0)  # bolt row angle
X_END = 25.0  # saddle end beyond A2/A4
T_PL = 6.0  # flange thickness (grip)
H_PL = 17.0  # half size of the flange (face is 38, keep >= 1 mm inside)
G_FRONT = math.radians(40.0)  # keel front face, from the flange's -W edge
G_BACK = math.radians(30.0)  # keel back face, leaning away from mount B
KEEL_L = 75.0  # keel length (runs into the pipe and is trimmed by it)
TAPER_DROP, TAPER_U = 45.0, 3.0  # keel's -U side narrows to U=3 over this drop
# print direction: world Z, which lies in the flange plane and is 40 deg off the pipe axis,
# so every face of the saddle and the flange is at most 40 deg from vertical
UP = rg.Vector3d(0, 0, 1)
Z_BED = -19.0  # bed plane (world Z), clear of the A1/A3 seat rings


def closed(brep, name):
    if brep is None:
        raise RuntimeError(name + ": no brep")
    if not brep.IsSolid:
        brep = brep.CapPlanarHoles(TOL) or brep
    if not brep.IsSolid:
        raise RuntimeError(name + ": not a closed solid")
    if brep.SolidOrientation == rg.BrepSolidOrientation.Inward:
        brep.Flip()
    return brep


def extrude(curve, vec, name):
    srf = rg.Surface.CreateExtrusion(curve, vec)
    return closed(rg.Brep.CreateFromSurface(srf), name)


def poly(pts):
    return rg.PolylineCurve(list(pts) + [pts[0]])


def saddle_prism():
    def yz(r, ang):
        return (r * math.sin(ang), r * math.cos(ang))

    x0 = -120.0
    rc = R_OUT / math.cos(PSI - PHI)
    pts = [yz(20.0, -PSI), yz(rc, -PSI), (0.0, R_OUT / math.cos(PHI)), yz(rc, PSI), yz(20.0, PSI)]
    crv = poly([LA(x0, a, b) for a, b in pts])
    return extrude(crv, AX * (X_END - x0), "saddle")


def flange_keel():
    cf = (-math.cos(G_FRONT), math.sin(G_FRONT))
    cb = (-math.cos(G_BACK), math.sin(G_BACK))
    wn = [(H_PL, 0.0), (H_PL, T_PL), (-H_PL, T_PL),
          (-H_PL + KEEL_L * cf[0], T_PL + KEEL_L * cf[1]),
          (-H_PL + KEEL_L * cb[0], KEEL_L * cb[1]), (-H_PL, 0.0)]
    crv = poly([LB(-H_PL, w, n) for w, n in wn])
    return extrude(crv, BU * (2 * H_PL), "flange_keel")


def taper_cutter2():
    # half space on the -U side of the taper plane, limited to below the flange edge
    o = LB(-H_PL, -H_PL, 0.0)
    t = rg.Vector3d(BU * (TAPER_U + H_PL) - BW * TAPER_DROP)
    t.Unitize()
    pl = rg.Plane(o, t, BN)  # z = t x N
    sgn = 1.0 if pl.ZAxis * BU < 0 else -1.0
    zi = rg.Interval(0, 200) if sgn > 0 else rg.Interval(-200, 0)
    return rg.Box(pl, rg.Interval(0, 300), rg.Interval(-150, 150), zi).ToBrep()


def bed_cutter():
    pl = rg.Plane(rg.Point3d(0, 0, Z_BED), UP)
    return rg.Box(pl, rg.Interval(-400, 400), rg.Interval(-400, 400), rg.Interval(-400, 0)).ToBrep()


def pipe():
    c = rg.Circle(rg.Plane(OA - AX * 200.0, AX), R_PIPE)
    return rg.Cylinder(c, 400.0).ToBrep(True, True)


def hole(bid, dia, back, front):
    b = BOLTS[bid]
    p = P(b["position"])
    a = V(b["axis"])
    a.Unitize()
    base = p - a * back
    cyl = rg.Cylinder(rg.Circle(rg.Plane(base, a), dia / 2.0), back + front)
    return cyl.ToBrep(True, True)


def union(breps):
    r = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("union gave %s pieces" % (len(r) if r else None))
    return r[0]


def diff(body, cutters, name):
    r = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    if not r or len(r) != 1:
        raise RuntimeError(name + " gave %s pieces" % (len(r) if r else None))
    return r[0]


def build():
    log("start")
    fk = diff(flange_keel(), [taper_cutter2()], "taper")
    log("taper ok")
    body = union([saddle_prism(), fk])
    log("union ok")
    body = diff(body, [pipe()], "pipe")
    log("pipe ok")
    body = diff(body, [bed_cutter()], "bed")
    log("bed ok")
    cutters = []
    for k in ("A1", "A2", "A3", "A4"):
        cutters.append(hole(k, 5.5, 3.0, T_SAD + 4.0))
    for k in ("B1", "B2", "B3", "B4"):
        cutters.append(hole(k, 4.5, 3.0, T_PL + 4.0))
    body = diff(body, cutters, "holes")
    log("holes ok")
    body.MergeCoplanarFaces(TOL)
    if not body.IsSolid:
        raise RuntimeError("result not solid")
    log("volume", round(body.GetVolume(), 1), "faces", body.Faces.Count)
    overhang_report(body)
    if os.environ.get("JB_DBG") or True:
        log("--- checker dir -x")
        overhang_report(body, -AX)
    return [body]


def overhang_report(body, u=None):
    """Per face: area facing down steeper than 45 deg for build direction u (bed excluded)."""
    u = u or UP
    mp = rg.MeshingParameters.QualityRenderMesh
    hmin = 1e9
    data = []
    for fi in range(body.Faces.Count):
        f = body.Faces[fi]
        ms = rg.Mesh.CreateFromBrep(f.DuplicateFace(False), mp)
        tris = []
        for m in ms or []:
            for i in range(m.Faces.Count):
                q = m.Faces[i]
                for (i0, i1, i2) in ((q.A, q.B, q.C),) + (((q.A, q.C, q.D),) if q.IsQuad else ()):
                    pa, pb, pc = m.Vertices[i0], m.Vertices[i1], m.Vertices[i2]
                    cr = rg.Vector3d.CrossProduct(rg.Vector3d(pb - pa), rg.Vector3d(pc - pa))
                    ar = cr.Length / 2.0
                    if ar <= 1e-9:
                        continue
                    cr.Unitize()
                    h = min(rg.Vector3d(p) * u for p in (pa, pb, pc))
                    hmin = min(hmin, h)
                    tris.append((cr * u, ar, h))
        data.append((fi, f, tris))
    total = 0.0
    for fi, f, tris in data:
        bad = sum(ar for d, ar, h in tris if d < -0.7072 and h - hmin > 0.2)
        if bad > 0.5:
            kind = "plane" if f.IsPlanar(0.01) else ("cyl" if f.IsCylinder(0.01) else "free")
            log("  overhang face", fi, kind, round(bad, 1), "mm2")
            total += bad
    lo = [fi for fi, f, tris in data if tris and min(t[2] for t in tris) - hmin < 0.2]
    for fi in lo:
        f = body.Faces[fi]
        ok, pl = f.TryGetPlane(0.01)
        if not ok or abs(pl.ZAxis * u) < 0.999:
            continue
        pts = [v.Location for v in f.DuplicateFace(False).Vertices]
        crv = f.OuterLoop.To3dCurve()
        pts = [crv.PointAt(t) for t in [crv.Domain.ParameterAt(i / 200.0) for i in range(201)]]
        best = 1e9
        for k in range(180):
            a = math.radians(k)
            d = pl.XAxis * math.cos(a) + pl.YAxis * math.sin(a)
            vals = [rg.Vector3d(p) * d for p in pts]
            best = min(best, max(vals) - min(vals))
        com = rg.VolumeMassProperties.Compute(body).Centroid
        cp = pl.ClosestPoint(com)
        cont = crv.Contains(cp, pl, 0.01)
        bb = crv.GetBoundingBox(pl)
        lc = pl.RemapToPlaneSpace(cp)[1]
        log("COM in plane", round(lc.X, 1), round(lc.Y, 1), "bed bbox", bb.Min, bb.Max, "contains", cont)
        log("bed face", fi, "area", round(rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area, 1),
            "min width", round(best, 1))
    down_len = 0.0
    for ei in range(body.Edges.Count):
        e = body.Edges[ei]
        fl = e.AdjacentFaces()
        if len(fl) != 2:
            continue
        L = e.GetLength()
        n_ok = 0
        for k in range(8):
            t = e.Domain.ParameterAt((k + 0.5) / 8.0)
            p = e.PointAt(t)
            tg = e.TangentAt(t)
            ns = []
            for fi in fl:
                f = body.Faces[fi]
                okc, uu, vv = f.ClosestPoint(p)
                nn = f.NormalAt(uu, vv)
                if f.OrientationIsReversed:
                    nn = -nn
                ns.append(nn)
            if ns[0] * u < -0.05 and ns[1] * u < -0.05 and abs(tg * u) < 0.7071:
                s2 = ns[0] + ns[1]
                if s2.Length < 1e-6:
                    continue
                s2.Unitize()
                if not body.IsPointInside(p + s2 * 0.05, 0.001, True) and body.IsPointInside(p - s2 * 0.05, 0.001, True):
                    n_ok += 1
        if n_ok:
            down_len += L * n_ok / 8.0
            log("  downward edge", ei, "faces", list(fl), "len", round(L * n_ok / 8.0, 1))
    log("downward edges total", round(down_len, 1))
    log("overhang total", round(total, 1), "bed-level faces", lo,
        "height", round(max(t[2] for _, _, tr in data for t in tr) - hmin, 1))


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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(MSG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n" + "\n".join(MSG) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
