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
PSI = math.radians(56.0)  # half angle of the seat arc
PHI = math.radians(35.0)  # bolt row angle
X_SAD = 25.0  # half length of the saddle along the pipe
T_PL = 6.0  # flange thickness (grip)
H_PL = 17.0  # half size of the flange (face is 38, keep >= 1 mm inside)
R_PL = 4.0  # flange corner radius


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


def saddle():
    def yz(r, ang):
        return (r * math.sin(ang), r * math.cos(ang))

    x0 = -X_SAD
    a0 = yz(R_PIPE, -PSI)
    a1 = yz(R_PIPE, PSI)
    am = yz(R_PIPE, 0.0)
    f1 = yz(R_OUT / math.cos(PSI - PHI), PSI)
    f0 = yz(R_OUT / math.cos(PSI - PHI), -PSI)
    ridge = (0.0, R_OUT / math.cos(PHI))
    pc = rg.PolyCurve()
    pc.Append(rg.Arc(LA(x0, *a0), LA(x0, *am), LA(x0, *a1)))
    pc.Append(rg.Line(LA(x0, *a1), LA(x0, *f1)))
    pc.Append(rg.Line(LA(x0, *f1), LA(x0, *ridge)))
    pc.Append(rg.Line(LA(x0, *ridge), LA(x0, *f0)))
    pc.Append(rg.Line(LA(x0, *f0), LA(x0, *a0)))
    if not pc.IsClosed:
        raise RuntimeError("saddle profile not closed")
    return extrude(pc, AX * (2 * X_SAD), "saddle")


def flange():
    pl = rg.Plane(OB, BU, BW)
    rect = rg.Rectangle3d(pl, rg.Interval(-H_PL, H_PL), rg.Interval(-H_PL, H_PL))
    crv = rect.ToNurbsCurve()
    crv = rg.Curve.CreateFilletCornersCurve(crv, R_PL, TOL, 0.1) or crv
    return extrude(crv, BN * T_PL, "flange")


def rect_curve(pts):
    return rg.PolylineCurve([pts[0], pts[1], pts[2], pts[3], pts[0]])


def neck():
    # end in the flange (mid thickness), a strip between the two bolt columns
    hu, w0, w1 = 6.0, -14.0, 14.0
    nb = T_PL * 0.5
    cb = [LB(-hu, w0, nb), LB(hu, w0, nb), LB(hu, w1, nb), LB(-hu, w1, nb)]
    # end in the saddle, below the ridge, between the bolt pairs
    hx, hy, zs = 9.0, 7.0, 43.0
    xc = -3.0
    # match corners: +U on the flange ~ +Y on the pipe, +W ~ +X
    cs = [LA(xc - hx, -hy, zs), LA(xc - hx, hy, zs), LA(xc + hx, hy, zs), LA(xc + hx, -hy, zs)]
    lofts = rg.Brep.CreateFromLoft([rect_curve(cb), rect_curve(cs)], rg.Point3d.Unset,
                                   rg.Point3d.Unset, rg.LoftType.Straight, False)
    b = lofts[0]
    b.Faces.SplitKinkyFaces(0.01, True)
    return closed(b, "neck")


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


def build():
    body = union([saddle(), flange(), neck()])
    cutters = []
    for k in ("A1", "A2", "A3", "A4"):
        cutters.append(hole(k, 5.5, 3.0, T_SAD + 4.0))
    for k in ("B1", "B2", "B3", "B4"):
        cutters.append(hole(k, 4.5, 3.0, T_PL + 4.0))
    r = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("holes gave %s pieces" % (len(r) if r else None))
    body = r[0]
    body.MergeCoplanarFaces(TOL)
    if not body.IsSolid:
        raise RuntimeError("result not solid")
    log("volume", round(body.GetVolume(), 1), "faces", body.Faces.Count)
    overhang_report(body)
    return [body]


def overhang_report(body):
    """Sample build directions; area facing down steeper than 45 deg (excluding the bed)."""
    mp = rg.MeshingParameters.QualityRenderMesh
    meshes = rg.Mesh.CreateFromBrep(body, mp)
    m = rg.Mesh()
    for x in meshes:
        m.Append(x)
    m.FaceNormals.ComputeFaceNormals()
    tris = []
    for i in range(m.Faces.Count):
        f = m.Faces[i]
        a, b, c = m.Vertices[f.A], m.Vertices[f.B], m.Vertices[f.C]
        cr = rg.Vector3d.CrossProduct(rg.Vector3d(b - a), rg.Vector3d(c - a))
        area = cr.Length / 2.0
        if area <= 0:
            continue
        cr.Unitize()
        cen = rg.Point3d((a.X + b.X + c.X) / 3, (a.Y + b.Y + c.Y) / 3, (a.Z + b.Z + c.Z) / 3)
        tris.append((cr, area, cen))
    res = []
    n = 1500
    g = math.pi * (3 - math.sqrt(5))
    for i in range(n):
        zz = 1 - 2 * (i + 0.5) / n
        rr = math.sqrt(1 - zz * zz)
        u = rg.Vector3d(rr * math.cos(g * i), rr * math.sin(g * i), zz)
        hmin = min(t[2].X * u.X + t[2].Y * u.Y + t[2].Z * u.Z for t in tris)
        bad = 0.0
        bed = 0.0
        for nrm, area, cen in tris:
            d = nrm * u
            if d < -0.7072:
                h = cen.X * u.X + cen.Y * u.Y + cen.Z * u.Z
                if h - hmin < 0.3:
                    bed += area
                else:
                    bad += area
        res.append((bad, -bed, u))
    res.sort(key=lambda t: (t[0], t[1]))
    for bad, bed, u in res[:6]:
        log("dir", "%.3f %.3f %.3f" % (u.X, u.Y, u.Z), "overhang", round(bad, 1), "bed", round(-bed, 1))


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
