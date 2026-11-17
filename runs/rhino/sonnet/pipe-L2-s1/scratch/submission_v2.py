#! python3
import json, os, math, traceback
HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")
import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
BOLTS = {b["id"]: b for b in POSE["resolved"]["bolts"]}
FACE = POSE["resolved"]["mounting_faces"][0]
TOL = 0.001
msgs = []

def V(a): return rg.Vector3d(a[0], a[1], a[2])
def P(a): return rg.Point3d(a[0], a[1], a[2])
def unit(v):
    v = rg.Vector3d(v); v.Unitize(); return v
def dot(a, b): return a * b
def cross(a, b): return rg.Vector3d.CrossProduct(a, b)

# ---- pipe
PO = rg.Point3d(21.0, 136.5, -7.0)
PD = unit(V([0.2487956429525063, 0.5900653629615055, 0.7680648380709507]))
PR = 34.0
R1 = unit(V(BOLTS["A1"]["axis"]))
R2 = unit(V(BOLTS["A3"]["axis"]))
BIS = unit(R1 + R2)
cosa = BIS * R1
G = 6.0          # grip / pad thickness on pipe
RT = PR + G
S_EDGE = 9.0     # pad extends this far past bolt axis (away side)
L_AX = 25.5      # half length along pipe axis

def away(ra, rb):
    t = rb - (rb * ra) * ra
    t.Unitize()
    return -t

def pipe_tent():
    t1 = away(R1, R2); t2 = away(R2, R1)
    pts3 = [rg.Point3d(0, 0, 0) + 0.0 * BIS,
            P(R1 * RT + t1 * S_EDGE), P(BIS * (RT / cosa)), P(R2 * RT + t2 * S_EDGE)]
    pts = [PO - PD * L_AX + (rg.Vector3d(p) if not isinstance(p, rg.Vector3d) else p) for p in pts3]
    pl = rg.Polyline(pts + [pts[0]])
    crv = pl.ToPolylineCurve()
    srf = rg.Surface.CreateExtrusion(crv, PD * (2 * L_AX))
    brep = srf.ToBrep().CapPlanarHoles(TOL)
    return brep

# ---- B frame
BO = P(FACE["origin"]); BN = unit(V(FACE["normal"]))
BW = rg.Vector3d(0, 0, 1)
BU = unit(cross(BW, BN)) * -1.0   # chosen so that (BW, BU, BN) is right-handed
if cross(BW, BU) * BN < 0: BU = -BU
def bw(x, y, z):  # x along BU, y along BW, z along BN
    return BO + BU * x + BW * y + BN * z

PLATE_T = 5.0
PLATE_H = 17.8
LEAN = math.radians(10.0)

def b_plate():
    # trapezoid in the B-frame (y,z): the -y wall leans inward so that, printed standing on the pipe-axis end face, it is not an overhang
    pts2 = [(-PLATE_H, 0.0), (PLATE_H, 0.0), (PLATE_H, PLATE_T), (-PLATE_H + PLATE_T * math.tan(LEAN), PLATE_T)]
    pts = [bw(-PLATE_H, p[0], p[1]) for p in pts2]
    pl = rg.Polyline(pts + [pts[0]])
    srf = rg.Surface.CreateExtrusion(pl.ToPolylineCurve(), BU * (2 * PLATE_H))
    return srf.ToBrep().CapPlanarHoles(TOL)

def clip(poly, f):
    out = []
    n = len(poly)
    for i in range(n):
        p = poly[i]; q = poly[(i + 1) % n]
        fp = f(p); fq = f(q)
        if fp >= 0: out.append(p)
        if (fp >= 0) != (fq >= 0):
            tt = fp / (fp - fq)
            out.append((p[0] + tt * (q[0] - p[0]), p[1] + tt * (q[1] - p[1])))
    return out

ARM_X = 7.0
ARM_ZMIN = 2.0
ARM_YMAX = -2.0
ARM_A1 = 4.0
ARM_TMIN = 36.0

def strut():
    # arm: prism along the B-plate x axis of a polygon in the (y,z) plane of the B frame,
    # bounded by planes parallel / perpendicular to the pipe axis so it prints standing on the pipe-axis end face
    dy = PD * BW; dz = PD * BN
    aB = (BO - PO) * PD
    Wp = cross(BU, PD)
    wy = Wp * BW; wz = Wp * BN
    tB = (BO - PO) * Wp
    msgs.append("dy %.3f dz %.3f aB %.2f wy %.3f wz %.3f tB %.2f" % (dy, dz, aB, wy, wz, tB))
    poly = [(-300, -300), (300, -300), (300, 300), (-300, 300)]
    cons = [lambda p: dy * p[0] + dz * p[1] + aB + L_AX,
            lambda p: ARM_A1 - (dy * p[0] + dz * p[1] + aB),
            lambda p: wy * p[0] + wz * p[1] + tB - ARM_TMIN,
            lambda p: p[1] - ARM_ZMIN,
            lambda p: ARM_YMAX - p[0]]
    for c in cons:
        poly = clip(poly, c)
    msgs.append("arm polygon (y,z): " + " ".join("(%.1f,%.1f)" % p for p in poly))
    pts = [bw(-ARM_X, p[0], p[1]) for p in poly]
    pl = rg.Polyline(pts + [pts[0]])
    srf = rg.Surface.CreateExtrusion(pl.ToPolylineCurve(), BU * (2 * ARM_X))
    return srf.ToBrep().CapPlanarHoles(TOL)

def cyl(center, axis, r, h0, h1):
    pl = rg.Plane(center + axis * h0, axis)
    c = rg.Cylinder(rg.Circle(pl, r), h1 - h0)
    return c.ToBrep(True, True)

def analyse(brep):
    for sgn in (1,):
        z = PD * sgn
        tot = 0.0; flat = {}
        for fi, f in enumerate(brep.Faces):
            dom_u = f.Domain(0); dom_v = f.Domain(1)
            n = 24; cnt = 0; hit = 0
            area = rg.AreaMassProperties.Compute(f.ToBrep()).Area
            for i in range(n):
                for j in range(n):
                    u = dom_u.ParameterAt((i + 0.5) / n); v = dom_v.ParameterAt((j + 0.5) / n)
                    if not f.IsPointOnFace(u, v) == rg.PointFaceRelation.Interior: continue
                    cnt += 1
                    nn = f.NormalAt(u, v)
                    if nn * (-z) > 0.7071 + 1e-6:
                        hit += 1
                        if fi == 10 and hit < 4:
                            pt = f.PointAt(u, v)
                            msgs.append("   hit pt axial %.1f rad %.1f n.d %.2f" % ((pt - PO) * PD, ((pt - PO) - PD * ((pt - PO) * PD)).Length, nn * PD))
            if cnt and hit:
                fb = f.DuplicateFace(False)
                am = rg.AreaMassProperties.Compute(fb)
                cc = am.Centroid
                ok, pu, pv = f.ClosestPoint(cc)
                nn = f.NormalAt(pu, pv)
                msgs.append("  dir %+d face %d area %.0f (%.0f) share %.2f centroid axial %.1f n.d %.2f type %s" % (sgn, fi, area, am.Area, hit / cnt, (cc - PO) * PD, nn * PD, "plane" if f.IsPlanar(0.001) else "curved"))
                tot += area * hit / cnt
        msgs.append("dir %+d total overhang area ~%.0f" % (sgn, tot))

def describe(brep):
    for fi, f in enumerate(brep.Faces):
        amp = rg.AreaMassProperties.Compute(f.ToBrep())
        c = amp.Centroid
        v = c - PO
        ok, u, vv = f.ClosestPoint(c)
        nn = f.NormalAt(u, vv)
        msgs.append("face %d area %.0f cen axial %.1f n.d %.2f n.R1 %.2f n.R2 %.2f n.BN %.2f %s" % (fi, amp.Area, v * PD, nn * PD, nn * R1, nn * R2, nn * BN, f.SurfaceType if hasattr(f,'SurfaceType') else ''))

def build():
    tent = pipe_tent(); plate = b_plate(); st = strut()
    for nm, bb in (("tent", tent), ("plate", plate), ("strut", st)):
        if bb.GetVolume() < 0: bb.Flip()
        msgs.append("%s vol %.1f solid %s" % (nm, bb.GetVolume(), bb.IsSolid))
    u = rg.Brep.CreateBooleanUnion([tent, plate, st], TOL)
    if not u: raise RuntimeError("union failed")
    body = u[0]
    tools = [cyl(PO - PD * 200, PD, PR, 0, 400)]
    for k in ("A1", "A2", "A3", "A4"):
        b = BOLTS[k]; ax = unit(V(b["axis"])); p = P(b["position"])
        tools.append(cyl(p, ax, 2.75, -12, G + 4))
    for k in ("B1", "B2", "B3", "B4"):
        b = BOLTS[k]; ax = unit(V(b["axis"])); p = P(b["position"])
        tools.append(cyl(p, ax, 2.25, -3, PLATE_T + 4))
    msgs.append("union solid=%s valid=%s vol=%.1f" % (body.IsSolid, body.IsValid, body.GetVolume()))
    res = body
    for t in tools:
        if t.GetVolume() < 0: t.Flip()
    for i, t in enumerate(tools):
        d = rg.Brep.CreateBooleanDifference([res], [t], TOL)
        if not d:
            raise RuntimeError("difference failed at tool %d" % i)
        res = d[0]
        msgs.append("after tool %d: solid=%s vol=%.1f" % (i, res.IsSolid, res.GetVolume()))
    res.MergeCoplanarFaces(TOL)
    analyse(res)
    msgs.append("solid=%s volume=%.1f faces=%d" % (res.IsSolid, res.GetVolume(), res.Faces.Count))
    return [res]

def main():
    if os.path.exists(OUT): os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build():
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT): raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(msgs) + "\n")

try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(msgs))
    if os.path.exists(OUT): os.remove(OUT)
