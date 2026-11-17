#! python3
"""pipe-L1-s1: saddle clamp on the pipe (A) + square pad on the flat mount (B),
joined by one straight rectangular arm routed between the B bolt key columns.
Built in a headless Rhino document with RhinoCommon, exported as output.step."""
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
PL = TASK["poses"][0]["placements"]
BOLTS = TASK["poses"][0]["resolved"]["bolts"]
TOL = 0.001
MSG = []


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


def col(R, i):
    return rg.Vector3d(R[0][i], R[1][i], R[2][i])


# frames
OA = P(PL["A"]["origin"]); RA = PL["A"]["rotation"]
xA, yA, zA = col(RA, 0), col(RA, 1), col(RA, 2)
OB = P(PL["B"]["origin"]); RB = PL["B"]["rotation"]
uB, vB, nB = col(RB, 0), col(RB, 1), col(RB, 2)

# design parameters
R_PIPE = 34.0
T_SAD = 8.0                # saddle wall
SAD_HALF_X = 28.5          # along the pipe
SAD_HALF_ANG = 50.0        # degrees each side
T_PAD = 7.0                # pad thickness = M4 grip
PAD = 35.6                 # pad size (1 mm+ inside the 38 mm face)
PAD_R = 4.0                # pad corner radius
GRIP_A = 5.5               # M5 grip: spot face depth below seat
SPOT_D = 11.0              # spot face dia (8.5 head + clearance)
ARM_W, ARM_D = 12.0, 18.0  # arm section
ARM_PAD = (-6.0, 2.0)      # arm axis point on pad top (u, v)
ARM_SAD = (-4.0, -6.0)     # arm axis point on saddle (x along pipe, angle deg)


def fromA(x, ang, r):
    a = math.radians(ang)
    return OA + xA * x + (zA * math.cos(a) + yA * math.sin(a)) * r


def fromB(u, v, h):
    return OB + uB * u + vB * v + nB * h


def extrude_closed(curve, vec):
    srf = rg.Surface.CreateExtrusion(curve, vec)
    sb = srf.ToBrep()
    sb.Faces.SplitKinkyFaces(math.radians(1), True)
    b = sb.CapPlanarHoles(TOL)
    if b is None:
        raise RuntimeError("cap failed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def one(breps, what):
    if breps is None or len(breps) == 0:
        raise RuntimeError(what + " failed")
    if len(breps) != 1:
        breps = sorted(breps, key=lambda b: -b.GetVolume())
        MSG.append("%s gave %d pieces, keeping largest" % (what, len(breps)))
    return breps[0]


def saddle():
    pln = rg.Plane(OA + xA * (-SAD_HALF_X), zA, yA)  # local x=cos dir, y=sin dir
    ro, ri = R_PIPE + T_SAD, R_PIPE
    a = math.radians(SAD_HALF_ANG)
    def pt(r, ang):
        return pln.PointAt(r * math.cos(ang), r * math.sin(ang))
    outer = rg.ArcCurve(rg.Arc(pt(ro, -a), pt(ro, 0), pt(ro, a)))
    inner = rg.ArcCurve(rg.Arc(pt(ri, a), pt(ri, 0), pt(ri, -a)))
    pc = rg.PolyCurve()
    pc.Append(outer)
    pc.Append(rg.LineCurve(pt(ro, a), pt(ri, a)))
    pc.Append(inner)
    pc.Append(rg.LineCurve(pt(ri, -a), pt(ro, -a)))
    return extrude_closed(pc, xA * (2 * SAD_HALF_X))


def pad():
    pln = rg.Plane(OB, uB, vB)
    h = PAD / 2.0
    rect = rg.Rectangle3d(pln, rg.Interval(-h, h), rg.Interval(-h, h))
    crv = rect.ToNurbsCurve()
    crvs = rg.Curve.CreateFilletCornersCurve(crv, PAD_R, TOL, math.radians(1))
    return extrude_closed(crvs if crvs else crv, nB * T_PAD)


def arm():
    q2 = fromB(ARM_PAD[0], ARM_PAD[1], T_PAD)
    q1 = fromA(ARM_SAD[0], ARM_SAD[1], R_PIPE + 4.0)
    d = q2 - q1; L = d.Length; d.Unitize()
    ew = rg.Vector3d.CrossProduct(d, nB); ew.Unitize()
    ed = rg.Vector3d.CrossProduct(ew, d); ed.Unitize()
    pln = rg.Plane(q1 - d * 12.0, ed, ew)  # normal = d
    box = rg.Box(pln, rg.Interval(-ARM_D / 2, ARM_D / 2), rg.Interval(-ARM_W / 2, ARM_W / 2),
                 rg.Interval(0, L + 12.0 + 25.0)).ToBrep()
    MSG.append("arm: %.1f deg off B normal, length %.1f" % (math.degrees(math.acos(-d * nB)), L))
    # trim: stay inside the saddle wall (r >= 36) and inside the pad (h >= 0.5)
    cyl = rg.Cylinder(rg.Circle(rg.Plane(OA - xA * 200, xA), R_PIPE + 2.0), 400).ToBrep(True, True)
    hb = rg.Box(rg.Plane(OB, uB, vB), rg.Interval(-200, 200), rg.Interval(-200, 200),
                rg.Interval(-200, 0.5)).ToBrep()
    return one(rg.Brep.CreateBooleanDifference([box], [cyl, hb], TOL), "arm trim")


def cutters():
    cs = []
    for b in BOLTS:
        p = P(b["position"]); a = V(b["axis"]); a.Unitize()
        lo, hi = b["clearance_diameter_range"]
        dia = round((lo + hi) / 2.0, 2)
        if b["id"].startswith("A"):
            cs.append(rg.Cylinder(rg.Circle(rg.Plane(p - a * 3, a), dia / 2), GRIP_A + 6).ToBrep(True, True))
            cs.append(rg.Cylinder(rg.Circle(rg.Plane(p + a * GRIP_A, a), SPOT_D / 2), 12).ToBrep(True, True))
        else:
            cs.append(rg.Cylinder(rg.Circle(rg.Plane(p - a * 3, a), dia / 2), T_PAD + 6).ToBrep(True, True))
    return cs


def build():
    body = one(rg.Brep.CreateBooleanUnion([saddle(), pad(), arm()], TOL), "union")
    body = one(rg.Brep.CreateBooleanDifference([body], cutters(), TOL), "holes")
    body.MergeCoplanarFaces(TOL)
    body.Faces.SplitKinkyFaces(math.radians(1), True)
    kinds = {}
    for f in body.Faces:
        k = "plane" if f.IsPlanar(TOL) else ("cyl" if f.IsCylinder(TOL) else "other")
        kinds[k] = kinds.get(k, 0) + 1
    MSG.append("face kinds %s" % kinds)
    for f in body.Faces:
        amp = rg.AreaMassProperties.Compute(f)
        c = amp.Centroid
        k = "plane" if f.IsPlanar(TOL) else ("cyl" if f.IsCylinder(TOL) else "other")
        MSG.append("  %-5s area %8.1f  A(%.1f,%.1f,%.1f) B(%.1f,%.1f,%.1f) loops %d" % (k, amp.Area,
            (c - OA) * xA, (c - OA) * yA, (c - OA) * zA, (c - OB) * uB, (c - OB) * vB, (c - OB) * nB, f.Loops.Count))
    MSG.append("solid=%s valid=%s faces=%d volume=%.0f" % (body.IsSolid, body.IsValid, body.Faces.Count, body.GetVolume()))
    return [body]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.ModelAbsoluteTolerance = TOL
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
    open(LOG, "w").write("error:\n" + err + "\n" + "\n".join(MSG))
    if os.path.exists(OUT):
        os.remove(OUT)
