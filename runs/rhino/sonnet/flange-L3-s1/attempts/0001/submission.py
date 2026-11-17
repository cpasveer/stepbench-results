#! python3
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

# ---- parameters
T_A = 4.0      # plate thickness at A
T_B = 5.0      # plate thickness at B
R_BOSS_A = 6.0
W_BAR_A = 9.0
R_HUB_A = 14.5
R_BOSS_B = 6.5
W_BAR_B = 10.0
W_SPINE_B = 12.0
STRUT_W = 10.0
STRUT_Z0, STRUT_Z1 = -6.0, 12.0
HOLE_D = {"M5": 5.5, "M6": 6.7}


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


def frame(face):
    o = P(face["origin"])
    n = V(face["normal"]); n.Unitize()
    z = rg.Vector3d(0, 0, 1)
    t = rg.Vector3d.CrossProduct(z, n)  # so that t x z = n
    t.Unitize()
    # plane x=t, y=z, normal = t x z
    pl = rg.Plane(o, t, z)
    if rg.Vector3d.Multiply(pl.Normal, n) < 0:
        pl = rg.Plane(o, -t, z)
    return pl, n


def to_solid(curve, plane, dist):
    """extrude closed planar curve along plane normal by dist (can be negative)."""
    ext = rg.Extrusion.Create(curve, dist, True)
    b = ext.ToBrep()
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def circle_curve(pl, u, v, r):
    p = pl.PointAt(u, v)
    return rg.Circle(rg.Plane(p, pl.XAxis, pl.YAxis), r).ToNurbsCurve()


def rect_curve(pl, u0, v0, u1, v1):
    pts = [pl.PointAt(u0, v0), pl.PointAt(u1, v0), pl.PointAt(u1, v1), pl.PointAt(u0, v1)]
    pts.append(pts[0])
    return rg.PolylineCurve(pts)


def bar_curve(pl, a, b, w):
    ax = rg.Vector3d(b[0] - a[0], b[1] - a[1], 0)
    L = ax.Length
    ax.Unitize()
    nx, ny = -ax.Y, ax.X
    h = w / 2.0
    pts = [pl.PointAt(a[0] + nx * h, a[1] + ny * h), pl.PointAt(b[0] + nx * h, b[1] + ny * h),
           pl.PointAt(b[0] - nx * h, b[1] - ny * h), pl.PointAt(a[0] - nx * h, a[1] - ny * h)]
    pts.append(pts[0])
    return rg.PolylineCurve(pts)


def union(breps):
    res = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("union failed: %s" % (None if not res else len(res)))
    return res[0]


def local_uv(pl, pt):
    d = pt - pl.Origin
    return (rg.Vector3d.Multiply(d, pl.XAxis), rg.Vector3d.Multiply(d, pl.YAxis))


def build():
    plA, nA = frame(FACES["A"])
    plB, nB = frame(FACES["B"])
    # ---------- A plate
    cA = plA.Origin
    # A bolt local coordinates
    abolts = {}
    for k in range(1, 7):
        uv = local_uv(plA, P(BOLTS["A%d" % k]["position"]))
        abolts[k] = uv
    parts = []
    parts.append(to_solid(circle_curve(plA, 0, 0, R_HUB_A), plA, T_A))
    for k in abolts:
        u, v = abolts[k]
        parts.append(to_solid(circle_curve(plA, u, v, R_BOSS_A), plA, T_A))
    pairs = [(1, 4), (3, 6), (1, 2), (2, 3), (4, 5), (5, 6)]
    for a, b in pairs:
        parts.append(to_solid(bar_curve(plA, abolts[a], abolts[b], W_BAR_A), plA, T_A))
    plate_a = union(parts)

    # ---------- B plate
    bb = {}
    for k in range(1, 5):
        bb[k] = local_uv(plB, P(BOLTS["B%d" % k]["position"]))
    parts = []
    for k in bb:
        u, v = bb[k]
        parts.append(to_solid(circle_curve(plB, u, v, R_BOSS_B), plB, T_B))
    # bars joining bosses in tB direction (bottom pair, top pair)
    # find pairs sharing v
    ks = sorted(bb.keys(), key=lambda k: (round(bb[k][1]), bb[k][0]))
    lo = [k for k in bb if bb[k][1] < 0]
    hi = [k for k in bb if bb[k][1] > 0]
    for grp in (lo, hi):
        a, b = grp
        parts.append(to_solid(bar_curve(plB, bb[a], bb[b], W_BAR_B), plB, T_B))
    vlo = bb[lo[0]][1]; vhi = bb[hi[0]][1]
    parts.append(to_solid(rect_curve(plB, -W_SPINE_B / 2, vlo, W_SPINE_B / 2, vhi), plB, T_B))
    plate_b = union(parts)

    # ---------- strut (z-prism in plan)
    def plan(p):
        return (p.X, p.Y)
    oa = plan(plA.Origin); ob = plan(plB.Origin)
    na = (nA.X, nA.Y); nbv = (nB.X, nB.Y)
    # axis end points on planes nA=3 (A) and nB=3 (B)
    sa = (oa[0] + 3 * na[0], oa[1] + 3 * na[1])
    sb = (ob[0] + 3 * nbv[0], ob[1] + 3 * nbv[1])
    d = (sb[0] - sa[0], sb[1] - sa[1]); L = math.hypot(*d); d = (d[0] / L, d[1] / L)
    p = (-d[1], d[0])
    h = STRUT_W / 2.0

    def hit(base, side, o, n, off):
        # point base+side*p*h + s*d on plane n.(x-o)=off
        q = (base[0] + side * h * p[0], base[1] + side * h * p[1])
        den = n[0] * d[0] + n[1] * d[1]
        s = (off - ((q[0] - o[0]) * n[0] + (q[1] - o[1]) * n[1])) / den
        return (q[0] + s * d[0], q[1] + s * d[1])
    a1 = hit(sa, +1, oa, na, 3.0); a2 = hit(sa, -1, oa, na, 3.0)
    b1 = hit(sb, +1, ob, nbv, 3.0); b2 = hit(sb, -1, ob, nbv, 3.0)
    pts = [rg.Point3d(a1[0], a1[1], STRUT_Z0), rg.Point3d(b1[0], b1[1], STRUT_Z0),
           rg.Point3d(b2[0], b2[1], STRUT_Z0), rg.Point3d(a2[0], a2[1], STRUT_Z0)]
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    ext = rg.Extrusion.Create(crv, -(STRUT_Z1 - STRUT_Z0), True)
    strut = ext.ToBrep()
    if strut.SolidOrientation == rg.BrepSolidOrientation.Inward:
        strut.Flip()

    body = union([plate_a, plate_b, strut])

    # ---------- holes
    cutters = []
    for bid, b in BOLTS.items():
        pos = P(b["position"]); ax = V(b["axis"]); ax.Unitize()
        d_h = HOLE_D[b["size"]]
        base = pos - ax * 1.0
        pl = rg.Plane(base, ax)
        c = rg.Circle(pl, d_h / 2.0).ToNurbsCurve()
        cyl = rg.Extrusion.Create(c, 12.0, True).ToBrep()
        if cyl.SolidOrientation == rg.BrepSolidOrientation.Inward:
            cyl.Flip()
        cutters.append(cyl)
    res = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("difference failed")
    body = res[0]
    body.MergeCoplanarFaces(TOL)
    return [body]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    msg = ""
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build():
            msg += "valid=%s solid=%s vol=%.1f faces=%d\n" % (brep.IsValid, brep.IsSolid, brep.GetVolume(), brep.Faces.Count)
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + msg)


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
