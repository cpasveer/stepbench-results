#! python3
"""Pipe-to-plate bracket, printed standing on its pipe-axis end face (no supports).

A saddle block sits on the pipe and carries the four M5 bolts on two flat bearing planes
(tent-shaped roof). An arm leaves the saddle toward the square mount; a 5 mm plate on the
mount face carries the four M4 bolts. Everything except the plate is a prism along the pipe
axis, so it prints as vertical walls; a small wedge under the plate's low edge keeps the
tilted plate supported.
"""
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
def cross(a, b): return rg.Vector3d.CrossProduct(a, b)

# ---------------- fixed geometry
PO = rg.Point3d(21.0, 136.5, -7.0)
PD = unit(V([0.2487956429525063, 0.5900653629615055, 0.7680648380709507]))   # pipe axis (exact)
PR = 34.0
R1 = unit(V(BOLTS["A1"]["axis"]))
R2 = unit(V(BOLTS["A3"]["axis"]))
BIS = unit(R1 + R2)
cosa = BIS * R1

BO = P(FACE["origin"]); BN = unit(V(FACE["normal"]))
BW = rg.Vector3d(0, 0, 1)
BU = unit(cross(BW, BN)) * -1.0
if cross(BW, BU) * BN < 0: BU = -BU
Wp = cross(BU, PD)            # in-slice direction, roughly from pipe toward the mount

def bw(x, y, z):  # mount frame: x along BU, y along BW (up), z along BN (out of the face)
    return BO + BU * x + BW * y + BN * z

# ---------------- parameters
G = 6.0               # pad thickness on the pipe (grip)
RT = PR + G
S_EDGE = 9.0          # pad extends this far past the bolt axis
L_AX = 25.5           # half length along the pipe axis
PLATE_T = 5.0
PLATE_H = 17.8        # plate half size (face is +-19, seat must stay 1 mm inside)
ARM_W = 14.0
ARM_ZMIN = 2.0        # arm stays this far off the mount face
ARM_ANGLE = 66.0      # degrees, direction of the first arm segment (rotated from the saddle bisector)
ARM_LA = 30.0
ARM_TEND = 94.0       # slice t where the arm ends over the plate

def slice_pt(s, t, axial):
    return PO + BU * s + Wp * t + PD * axial

def rot_in_slice(vec, deg):
    v = rg.Vector3d(vec)
    v.Rotate(math.radians(deg), PD)
    return v

def away(ra, rb):
    t = rb - (rb * ra) * ra
    t.Unitize()
    return -t

def poly_curve(pts):
    return rg.Polyline(list(pts) + [pts[0]]).ToPolylineCurve()

def tent_curve():
    t1 = away(R1, R2); t2 = away(R2, R1)
    v = [rg.Vector3d(0, 0, 0), R1 * RT + t1 * S_EDGE, BIS * (RT / cosa), R2 * RT + t2 * S_EDGE]
    return poly_curve([PO - PD * L_AX + x for x in v])

def rect_curve(p0, direction, length, width, back=0.0):
    perp = cross(PD, direction); perp.Unitize()
    a = p0 - direction * back
    b = p0 + direction * length
    return poly_curve([a + perp * (width / 2), b + perp * (width / 2), b - perp * (width / 2), a - perp * (width / 2)])

def solid_from_curve(crv, vec):
    srf = rg.Surface.CreateExtrusion(crv, vec)
    br = srf.ToBrep().CapPlanarHoles(TOL)
    if br.GetVolume() < 0: br.Flip()
    return br

def main_body():
    base = PO - PD * L_AX
    S0 = base + BIS * 39.0
    dirA = unit(rot_in_slice(BIS, ARM_ANGLE - 60.0))
    P1 = S0 + dirA * ARM_LA
    # target over the plate
    centre = BO - PO
    s_c = centre * BU
    P2 = base + BU * s_c + Wp * ARM_TEND
    P2 = rg.Point3d(P2)
    segB = P2 - P1
    segB = rg.Vector3d(segB - PD * (segB * PD)); lenB = segB.Length; dirB = unit(segB)
    msgs.append("dirA vs bis %.1f deg, bend %.1f deg, lenB %.1f" % (math.degrees(math.acos(max(-1, min(1, dirA * BIS)))), math.degrees(math.acos(max(-1, min(1, dirA * dirB)))), lenB))
    curves = [tent_curve(), rect_curve(S0, dirA, ARM_LA, ARM_W, back=3.0), rect_curve(P1, dirB, lenB, ARM_W, back=ARM_W / 2)]
    un = rg.Curve.CreateBooleanUnion(curves, TOL)
    if not un or len(un) != 1: raise RuntimeError("profile union gave %s curves" % (0 if not un else len(un)))
    body = solid_from_curve(un[0], PD * (2 * L_AX))
    # keep the arm off the mount face
    pl = rg.Plane(BO, BW, BU)   # z axis = BN
    below = rg.Box(pl, rg.Interval(-400, 400), rg.Interval(-400, 400), rg.Interval(-300, ARM_ZMIN)).ToBrep()
    if below.GetVolume() < 0: below.Flip()
    d = rg.Brep.CreateBooleanDifference([body], [below], TOL)
    if not d: raise RuntimeError("arm trim failed")
    return d[0]

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

def extrude_yz(poly, x0, x1):
    pts = [bw(x0, p[0], p[1]) for p in poly]
    return solid_from_curve(poly_curve(pts), BU * (x1 - x0))

def plate():
    return extrude_yz([(-PLATE_H, 0.0), (PLATE_H, 0.0), (PLATE_H, PLATE_T), (-PLATE_H, PLATE_T)], -PLATE_H, PLATE_H)

def wedge():
    # sweep of the plate's +y end along the pipe axis down to the bed: carries the plate's lowest edge
    dy = PD * BW; dz = PD * BN
    aB = (BO - PO) * PD
    ax = lambda p: aB + dy * p[0] + dz * p[1]
    y0 = 16.9
    K = (L_AX - ax((PLATE_H, 0.0))) / 1.0
    poly = [(y0, 0.0), (PLATE_H, 0.0), (PLATE_H + dy * K, dz * K)]
    # bed line from there up to y = y0
    zD = (L_AX - aB - dy * y0) / dz
    poly.append((y0, zD))
    msgs.append("wedge poly " + " ".join("(%.1f,%.1f)" % p for p in poly))
    return extrude_yz(poly, -PLATE_H, PLATE_H)

def cyl(center, axis, r, h0, h1):
    pl = rg.Plane(center + axis * h0, axis)
    c = rg.Cylinder(rg.Circle(pl, r), h1 - h0)
    b = c.ToBrep(True, True)
    if b.GetVolume() < 0: b.Flip()
    return b

def analyse(brep, sgn):
    z = PD * sgn
    tot = 0.0
    for fi, f in enumerate(brep.Faces):
        dom_u = f.Domain(0); dom_v = f.Domain(1)
        n = 24; cnt = 0; hit = 0
        area = rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area
        for i in range(n):
            for j in range(n):
                u = dom_u.ParameterAt((i + 0.5) / n); v = dom_v.ParameterAt((j + 0.5) / n)
                if f.IsPointOnFace(u, v) != rg.PointFaceRelation.Interior: continue
                cnt += 1
                nn = f.NormalAt(u, v)
                if nn * (-z) > 0.7071 + 1e-6:
                    hit += 1
                    if fi == 2 and hit % 40 == 1:
                        pt = f.PointAt(u, v)
                        msgs.append("   hit pt B-frame %.1f %.1f %.1f axial %.1f n.d %.2f" % ((pt - BO) * BU, (pt - BO) * BW, (pt - BO) * BN, (pt - PO) * PD, nn * PD))
        if cnt and hit:
            fb = f.DuplicateFace(False)
            cc = rg.AreaMassProperties.Compute(fb).Centroid
            msgs.append("  face %d area %.0f share %.2f axial %.1f %s" % (fi, area, hit / cnt, (cc - PO) * PD, "plane" if f.IsPlanar(0.001) else "curved"))

def build():
    parts = [main_body(), plate(), wedge()]
    for p in parts:
        msgs.append("part vol %.1f solid %s" % (p.GetVolume(), p.IsSolid))
    u = rg.Brep.CreateBooleanUnion(parts, TOL)
    if not u or len(u) != 1: raise RuntimeError("union failed")
    body = u[0]
    tools = [cyl(PO - PD * 200, PD, PR, 0, 400)]
    for k in ("A1", "A2", "A3", "A4"):
        b = BOLTS[k]; tools.append(cyl(P(b["position"]), unit(V(b["axis"])), 2.75, -12, G + 4))
    for k in ("B1", "B2", "B3", "B4"):
        b = BOLTS[k]; tools.append(cyl(P(b["position"]), unit(V(b["axis"])), 2.25, -3, PLATE_T + 4))
    res = body
    for i, t in enumerate(tools):
        d = rg.Brep.CreateBooleanDifference([res], [t], TOL)
        if not d: raise RuntimeError("difference failed at tool %d" % i)
        res = d[0]
    res.MergeCoplanarFaces(TOL)
    msgs.append("solid=%s valid=%s volume=%.1f faces=%d" % (res.IsSolid, res.IsValid, res.GetVolume(), res.Faces.Count))
    analyse(res, -1)
    for fi, f in enumerate(res.Faces):
        bb = f.GetBoundingBox(True)
        sf = f.UnderlyingSurface()
        msgs.append("F%d %s area %.0f axial %.1f..%.1f" % (fi, sf.GetType().Name, rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area, (bb.Min - PO) * PD if False else min((bb.Corner(i % 2 == 0, (i // 2) % 2 == 0, i // 4 == 0) - PO) * PD for i in range(8)), max((bb.Corner(i % 2 == 0, (i // 2) % 2 == 0, i // 4 == 0) - PO) * PD for i in range(8))))
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
    open(LOG, "w").write("error:\n" + err + "\n" + "\n".join(msgs))
    if os.path.exists(OUT): os.remove(OUT)
