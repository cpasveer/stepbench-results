#! python3
"""pipe-L2-s1: saddle-to-flange bracket, modelled with RhinoCommon.

Layout (local frame: Z = pipe axis, X = towards mount B, Y = B's in-plane
direction that is square to the pipe):
  * a saddle on the pipe: inner face r = 34, outside three flats (one square to
    each bolt row, one between them where the web lands);
  * a flange on B: 6 mm plate inside B's outline, with a foot down to the
    print bed;
  * a web between them that passes between the B bolt columns and between
    the two pipe bolt rows, so every head stays open for its hex key.
Printed standing on the +Z end face (pipe axis vertical): the saddle and web
are plain vertical prisms, the flange leans 40 deg, nothing needs support.
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
BOLTS = POSE["resolved"]["bolts"]
TOL = 0.001
MSG = []

# ---- design numbers (mm) ----
R_PIPE = 34.0
T_SAD = 6.0        # saddle thickness = grip of the M5 bolts
Z_HALF = 24.0      # saddle half length along the pipe
PAD = 9.0          # saddle flat beyond a bolt axis
T_PL = 6.0         # flange thickness = grip of the M4 bolts
HALF_PL = 17.7     # flange half size (B face is 38: 1.3 mm inside its edge)
T_FOOT = 4.0       # foot wall under the flange
W_NECK = 12.0      # web thickness between the B bolt columns
LAND = 10.0        # web half width where it lands on the saddle
X_KINK = 64.0      # where the web turns towards the saddle
R_KINK = 20.0
D_A = 5.5
D_B = 4.5
OVER = 30.0        # overshoot, cut back by the end planes


def v3(a):
    return rg.Vector3d(a[0], a[1], a[2])


def p3(a):
    return rg.Point3d(a[0], a[1], a[2])


def frames():
    pa = POSE["placements"]["A"]
    pb = POSE["placements"]["B"]
    ra, rb = pa["rotation"], pb["rotation"]
    col = lambda r, i: rg.Vector3d(r[0][i], r[1][i], r[2][i])
    O = p3(pa["origin"])
    eZ = col(ra, 0)                       # pipe axis
    n = col(rb, 2)                        # B outward normal
    zw = col(rb, 0)
    ub = col(rb, 1)
    Bc = p3(pb["origin"])
    MSG.append("ub.axis=%.2e" % (ub * eZ))
    eY = ub - eZ * (ub * eZ)
    eY.Unitize()
    eX = rg.Vector3d.CrossProduct(eY, eZ)
    if (Bc - O) * eX < 0:
        eX = -eX
        eY = -eY
    loc = lambda v: rg.Vector3d(v * eX, v * eY, v * eZ)
    return O, eX, eY, eZ, loc(Bc - O), loc(n), loc(zw), loc


def prism(curve, path_from, path_to):
    face = rg.Brep.CreatePlanarBreps(curve, TOL)
    if not face or len(face) != 1:
        raise RuntimeError("profile is not a single planar region")
    b = face[0].Faces[0].CreateExtrusion(rg.LineCurve(path_from, path_to), True)
    if b is None or not b.IsSolid:
        raise RuntimeError("extrusion failed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cut(brep, origin, discard_dir):
    """Remove everything on the discard_dir side of the plane, close the cut."""
    parts = brep.Trim(rg.Plane(origin, discard_dir), TOL)
    if not parts or len(parts) != 1:
        raise RuntimeError("plane cut gave %s pieces" % (len(parts) if parts else 0))
    b = parts[0].CapPlanarHoles(TOL)
    if b is None or not b.IsSolid:
        raise RuntimeError("cap failed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def union(a, b):
    r = rg.Brep.CreateBooleanUnion([a, b], TOL)
    if not r or len(r) != 1:
        raise RuntimeError("union gave %s pieces" % (len(r) if r else 0))
    return r[0]


def build():
    O, eX, eY, eZ, Bc, n, zw, loc = frames()
    MSG.append("Bc=%s n=%s zw=%s" % (Bc, n, zw))
    if not (abs(n.Y) < 1e-6 and n.Z > 0.3 and n.X < 0 and abs(zw.Y) < 1e-6):
        raise RuntimeError("layout is not the one this script was drawn for")
    if zw.Z < 0:
        zw = -zw
    yc = Bc.Y

    # ---------- saddle (prism along Z) ----------
    rad = {}
    for b in BOLTS:
        if b["id"].startswith("A"):
            a = loc(v3(b["axis"]))
            rad[(round(a.X, 4), round(a.Y, 4))] = rg.Vector3d(a.X, a.Y, 0)
    r1, r2 = list(rad.values())
    r1.Unitize(); r2.Unitize()
    if r1.X * r2.Y - r1.Y * r2.X < 0:
        r1, r2 = r2, r1                       # r1 -> r2 anticlockwise
    bis = r1 + r2
    bis.Unitize()
    tm = rg.Vector3d(-bis.Y, bis.X, 0)
    half = math.acos(r1 * bis)                # 35 deg
    t1 = rg.Vector3d(-r1.Y, r1.X, 0)
    t2 = rg.Vector3d(-r2.Y, r2.X, 0)
    d_out = R_PIPE + T_SAD
    o = rg.Point3d(0, 0, 0)
    rin = math.sqrt(R_PIPE ** 2 - PAD ** 2)
    a_start = o + r1 * rin - t1 * PAD
    a_end = o + r2 * rin + t2 * PAD
    a_mid = o + bis * R_PIPE
    e1 = o + r1 * d_out - t1 * PAD
    e2 = o + r2 * d_out + t2 * PAD
    rc = d_out / math.cos(half / 2.0)
    c1 = o + (bis * math.cos(half / 2.0) - tm * math.sin(half / 2.0)) * rc
    c2 = o + (bis * math.cos(half / 2.0) + tm * math.sin(half / 2.0)) * rc
    arc = rg.Arc(a_start, a_mid, a_end)
    prof = rg.PolyCurve()
    prof.Append(rg.ArcCurve(arc))
    for a, b in ((a_end, e2), (e2, c2), (c2, c1), (c1, e1), (e1, a_start)):
        prof.Append(rg.LineCurve(a, b))
    zv = rg.Vector3d(0, 0, 1)
    prof.Translate(zv * -Z_HALF)
    saddle = prism(prof, rg.Point3d(0, 0, -Z_HALF), rg.Point3d(0, 0, OVER))

    # ---------- flange + foot (prism along Y) ----------
    def T(s, h):
        return rg.Point3d(Bc.X + zw.X * s + n.X * h, 0, Bc.Z + zw.Z * s + n.Z * h)
    T1 = T(-HALF_PL, 0)
    T2 = T(HALF_PL, 0)
    z_top = T1.Z
    s_a = (-OVER - Bc.Z) / zw.Z
    s_b = (-OVER - Bc.Z - n.Z * T_PL) / zw.Z
    A0, A1 = T(s_a, 0), T(s_b, T_PL)
    x_in = T2.X - T_FOOT
    s_q = (x_in - Bc.X - n.X * T_PL) / zw.X
    Q = T(s_q, T_PL)
    pts = [A0, T2, rg.Point3d(T2.X, 0, OVER), rg.Point3d(x_in, 0, OVER), Q, A1, A0]
    y0, y1 = yc - HALF_PL, yc + HALF_PL
    pl = rg.PolylineCurve([rg.Point3d(p.X, y0, p.Z) for p in pts])
    flange = prism(pl, rg.Point3d(0, y0, 0), rg.Point3d(0, y1, 0))

    # ---------- web (prism along Z) ----------
    x_end = T2.X - T_FOOT / 2.0
    fa = o + bis * d_out - tm * LAND
    fb = o + bis * d_out + tm * LAND
    ka = rg.Point3d(X_KINK, yc - W_NECK / 2.0, 0)
    kb = rg.Point3d(X_KINK, yc + W_NECK / 2.0, 0)
    da = fa - ka; da.Unitize()
    db = fb - kb; db.Unitize()
    wp = [rg.Point3d(x_end, ka.Y, 0), ka, fa + da * 3.0, fb + db * 3.0, kb,
          rg.Point3d(x_end, kb.Y, 0)]
    for p in (wp[2], wp[3]):
        r = math.hypot(p.X, p.Y)
        if not (R_PIPE + 1.0 < r < d_out - 0.5):
            raise RuntimeError("web root not inside the saddle: r=%.2f" % r)
    segs = [rg.LineCurve(wp[i], wp[i + 1]) for i in range(5)]

    def fillet(c0, c1):
        f = rg.Curve.CreateFilletCurves(c0, c0.PointAtStart, c1, c1.PointAtEnd, R_KINK,
                                        True, True, True, TOL, 0.01)
        if not f or len(f) != 1:
            raise RuntimeError("web fillet failed")
        return f[0]
    side_a = fillet(segs[0], segs[1])
    side_b = fillet(segs[3], segs[4])
    web_c = rg.Curve.JoinCurves([side_a, segs[2], side_b,
                                 rg.LineCurve(wp[5], wp[0])], TOL)
    if len(web_c) != 1 or not web_c[0].IsClosed:
        raise RuntimeError("web outline not closed")
    wc = web_c[0]
    wc.Translate(zv * -OVER)
    web = prism(wc, rg.Point3d(0, 0, -OVER), rg.Point3d(0, 0, OVER))
    # end the web inside the flange (mid thickness) so no faces coincide
    web = cut(web, rg.Point3d(Bc.X, Bc.Y, Bc.Z) + n * (T_PL / 2.0), -n)

    body = union(web, flange)
    body = cut(body, rg.Point3d(0, 0, z_top), -zv)        # top of web and flange
    body = union(body, saddle)
    body = cut(body, rg.Point3d(0, 0, Z_HALF), zv)        # print bed face
    body.MergeCoplanarFaces(TOL)

    # ---------- bolt holes ----------
    to_world = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, rg.Plane(O, eX, eY))
    body.Transform(to_world)
    for b in BOLTS:
        p, a = p3(b["position"]), v3(b["axis"])
        a.Unitize()
        isA = b["id"].startswith("A")
        d = D_A if isA else D_B
        circ = rg.Circle(rg.Plane(p - a * 3.0, a), d / 2.0)
        cyl = rg.Cylinder(circ, 3.0 + (T_SAD if isA else T_PL) + 6.0).ToBrep(True, True)
        r = rg.Brep.CreateBooleanDifference(body, cyl, TOL)
        if not r or len(r) != 1:
            raise RuntimeError("hole %s failed" % b["id"])
        body = r[0]
    body.MergeCoplanarFaces(TOL)
    body.Compact()
    return body


def report(b):
    el = sorted(e.GetLength() for e in b.Edges)
    ar = sorted(rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area for f in b.Faces)
    vol = rg.VolumeMassProperties.Compute(b).Volume
    naked = sum(1 for e in b.Edges if e.Valence != rg.EdgeAdjacency.Interior)
    MSG.append("valid=%s solid=%s faces=%d edges=%d naked=%d vol=%.0f" % (
        b.IsValid, b.IsSolid, b.Faces.Count, b.Edges.Count, naked, vol))
    MSG.append("shortest edges " + " ".join("%.3f" % x for x in el[:6]))
    MSG.append("smallest faces " + " ".join("%.2f" % x for x in ar[:6]))
    kinds = {}
    for f in b.Faces:
        s = f.UnderlyingSurface()
        k = "plane" if s.IsPlanar(TOL) else ("cyl" if s.IsCylinder(TOL) else type(s).__name__)
        kinds[k] = kinds.get(k, 0) + 1
    MSG.append("faces by kind %s" % kinds)
    for name in ("fixed_A.step", "fixed_B.step"):
        try:
            d2 = Rhino.RhinoDoc.CreateHeadless(None)
            d2.ModelUnitSystem = Rhino.UnitSystem.Millimeters
            ok = d2.Import(os.path.join(HERE, name))
            tot = 0.0
            cnt = 0
            for ob in d2.Objects:
                g = ob.Geometry
                if isinstance(g, rg.Extrusion):
                    g = g.ToBrep()
                if isinstance(g, rg.Brep):
                    cnt += 1
                    x = rg.Brep.CreateBooleanIntersection(b, g, TOL)
                    if x:
                        tot += sum(rg.VolumeMassProperties.Compute(i).Volume for i in x)
            MSG.append("%s import=%s breps=%d overlap=%.4f" % (name, ok, cnt, tot))
            d2.Dispose()
        except Exception as e:
            MSG.append("%s check error %s" % (name, e))


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    body = build()
    if not (body.IsValid and body.IsSolid):
        raise RuntimeError("result is not a valid closed solid")
    report(body)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.Objects.AddBrep(body)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(MSG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(MSG) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
