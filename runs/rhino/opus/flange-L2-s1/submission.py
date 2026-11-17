#! python3
"""flange-L2-s1: bracket joining round flange A (6x M5) and rectangular block B (4x M6).

Concept: an "H" in plan view, printed standing up (world +Z is the build direction,
the bed is the plane z = Z_BED):
  * A plate: 8 mm round flange plate whose lower part runs down to the bed as a
    teardrop tail (tangent flanks at THETA below horizontal, so no overhang > 45 deg).
    The seat on A is a 71 mm pad; the edge is chamfered back (shallow cone, 31 deg
    overhang) so the seat stays inside A's 74.5 mm face and nothing hangs flat.
  * B plate: 8 mm rectangular plate, 43 x 93 mm seat, standing on the bed.
  * Web: one 8 mm vertical web between the plate centres, standing on the bed, its top
    sloping from A down to B. Its line is chosen so every bolt head and hex key has
    room: the web runs between the two bolt columns of each plate.
All bolt heads sit on the flat back faces of the plates; the keys swing outwards.
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
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TOL = 0.001
LOGLINES = []


def log(*a):
    LOGLINES.append(" ".join(str(x) for x in a))


# ---------------------------------------------------------------- parameters
T_PLATE = 8.0          # plate thickness (= grip)
Z_BED = -64.5          # build plate level (B seat bottom, 1.5 mm inside B's face)
A_SEAT_R = 35.5        # seat pad radius on A (face radius 37.25)
A_CH_A = 2.5           # chamfer: radial run
A_CH_H = 1.5           # chamfer: axial depth
A_RA = A_SEAT_R + A_CH_A   # outer radius of the A plate (behind the seat)
A_RECESS = 2.5         # depth of the non-seating front of the tail
TAIL_HALF_AT_BED = 13.0
B_HALF_U = 21.5        # B seat half width (face 23)
B_TOP = 28.5           # B seat top (face top 30)
WEB_HALF = 4.0
WEB_UA = -5.0          # web centre offset on A's back face (along A's tA)
WEB_UB = 4.2           # web centre offset on B's back face (along B's tB)
WEB_TOP_A = 33.5       # web top height at A back face (world z)
WEB_TOP_B = 20.0       # web top height at B back face (world z)
HOLE_D = {"M5": 5.5, "M6": 6.6}
Z_LOW = Z_BED - 12.0   # everything is built down to here and trimmed at the bed


def V(*c):
    return rg.Vector3d(*c)


def P(*c):
    return rg.Point3d(*c)


class Frame:
    """Local frame of a mounting face: x = outward normal, u = horizontal in-face, z = world z."""

    def __init__(self, face):
        o = face["origin"]
        n = face["normal"]
        self.o = P(*o)
        self.n = V(n[0], n[1], 0.0)
        self.n.Unitize()
        self.t = V(-self.n.Y, self.n.X, 0.0)
        self.z = V(0, 0, 1)

    def pt(self, x, u, zw):
        """x along normal, u along t, zw = WORLD z."""
        p = self.o + self.n * x + self.t * u
        return P(p.X, p.Y, zw)

    def plane(self, x):
        return rg.Plane(self.o + self.n * x, self.t, self.z)


FA = Frame(FACES["A"])
FB = Frame(FACES["B"])
ZA = FA.o.Z


def solid_check(b, name):
    if b is None:
        raise RuntimeError(name + ": None")
    if not b.IsSolid:
        raise RuntimeError(name + ": not a closed solid")
    return b


def one(res, name):
    if res is None or len(res) == 0:
        raise RuntimeError(name + ": boolean failed")
    if len(res) != 1:
        raise RuntimeError("%s: %d pieces" % (name, len(res)))
    return solid_check(res[0], name)


def extrude_planar(curve, vec, name):
    srf = rg.Surface.CreateExtrusion(curve, vec)
    b = srf.ToBrep().CapPlanarHoles(TOL)
    b = solid_check(b, name)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def frustum(frame, x0, r0, x1, r1, name):
    c0 = rg.Circle(frame.plane(x0), r0).ToNurbsCurve()
    c1 = rg.Circle(frame.plane(x1), r1).ToNurbsCurve()
    # frame.plane is centred at the face origin in u but at world z = origin z: ok (centre)
    lofts = rg.Brep.CreateFromLoft([c0, c1], rg.Point3d.Unset, rg.Point3d.Unset,
                                   rg.LoftType.Straight, False)
    b = lofts[0].CapPlanarHoles(TOL)
    b = solid_check(b, name)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def box_local(frame, x0, x1, u0, u1, z0, z1):
    pts = [frame.pt(x0, u0, z0), frame.pt(x1, u0, z0), frame.pt(x1, u1, z0), frame.pt(x0, u1, z0),
           frame.pt(x0, u0, z1), frame.pt(x1, u0, z1), frame.pt(x1, u1, z1), frame.pt(x0, u1, z1)]
    b = rg.Brep.CreateFromBox(pts)
    return solid_check(b, "box")


# ---------------------------------------------------------------- A plate
def a_plate():
    # Teardrop outline in A's face plane (u, world z): arc above the tangent points,
    # tangent flanks below, reaching TAIL_HALF_AT_BED at the bed, vertical below the bed.
    zb_rel = Z_BED - ZA
    # find theta so that the tangent flank reaches +-TAIL_HALF_AT_BED at the bed
    lo, hi = 1.0, 60.0
    for _ in range(80):
        th = 0.5 * (lo + hi)
        r = math.radians(th)
        ut, zt = A_RA * math.cos(r), -A_RA * math.sin(r)
        u_bed = ut - math.tan(r) * (zt - zb_rel)
        if u_bed > TAIL_HALF_AT_BED:
            lo = th
        else:
            hi = th
    r = math.radians(th)
    log("A tail flank angle below horizontal: %.2f deg" % th)
    ut, zt = A_RA * math.cos(r), -A_RA * math.sin(r)
    pl = FA.plane(0.0)   # x along t (u), y along world z, origin at face centre
    def q(u, zr):
        return pl.PointAt(u, zr)
    arc = rg.Arc(q(ut, zt), q(0, A_RA), q(-ut, zt))
    segs = [rg.ArcCurve(arc),
            rg.LineCurve(q(-ut, zt), q(-TAIL_HALF_AT_BED, zb_rel)),
            rg.LineCurve(q(-TAIL_HALF_AT_BED, zb_rel), q(-TAIL_HALF_AT_BED, Z_LOW - ZA)),
            rg.LineCurve(q(-TAIL_HALF_AT_BED, Z_LOW - ZA), q(TAIL_HALF_AT_BED, Z_LOW - ZA)),
            rg.LineCurve(q(TAIL_HALF_AT_BED, Z_LOW - ZA), q(TAIL_HALF_AT_BED, zb_rel)),
            rg.LineCurve(q(TAIL_HALF_AT_BED, zb_rel), q(ut, zt))]
    crv = rg.Curve.JoinCurves(segs, TOL)
    if len(crv) != 1 or not crv[0].IsClosed:
        raise RuntimeError("A outline not closed")
    plate = extrude_planar(crv[0], FA.n * T_PLATE, "A prism")

    # front cutter: everything in front of x = A_RECESS outside the chamfer cone
    k = A_CH_A / A_CH_H
    xa, xb = -1.0, A_RECESS + 0.5
    cone = frustum(FA, xa, A_SEAT_R + k * xa, xb, A_SEAT_R + k * xb, "cone")
    blk = box_local(FA, xa - 0.5, A_RECESS, -70, 70, Z_LOW - 20, ZA + 70)
    cutter = one(rg.Brep.CreateBooleanDifference(blk, cone, TOL), "A cutter")
    plate = one(rg.Brep.CreateBooleanDifference(plate, cutter, TOL), "A plate")
    return plate


# ---------------------------------------------------------------- B plate
def b_plate():
    return box_local(FB, 0.0, T_PLATE, -B_HALF_U, B_HALF_U, Z_LOW, B_TOP)


# ---------------------------------------------------------------- web
def web():
    pa = FA.pt(T_PLATE, WEB_UA, 0)
    pb = FB.pt(T_PLATE, WEB_UB, 0)
    d = pb - pa
    L = d.Length
    d.Unitize()
    s = V(-d.Y, d.X, 0)
    ext = 4.0
    a0 = pa - d * ext
    b0 = pb + d * ext
    def ztop(p):
        t = ((p - pa) * d) / L
        return WEB_TOP_A + (WEB_TOP_B - WEB_TOP_A) * t
    # check the embedded ends stay inside the plate material (x between 0 and T)
    for p in (a0 + s * WEB_HALF, a0 - s * WEB_HALF):
        log("web end A: x_A = %.2f" % ((p - FA.o) * FA.n - 0.0))
    for p in (b0 + s * WEB_HALF, b0 - s * WEB_HALF):
        log("web end B: x_B = %.2f" % ((p - FB.o) * FB.n))
    c = [a0 - s * WEB_HALF, b0 - s * WEB_HALF, b0 + s * WEB_HALF, a0 + s * WEB_HALF]
    pts = [P(p.X, p.Y, Z_LOW) for p in c] + [P(p.X, p.Y, ztop(p)) for p in c]
    return solid_check(rg.Brep.CreateFromBox(pts), "web")


def holes():
    cyl = []
    for b in BOLTS:
        ax = V(*b["axis"])
        ax.Unitize()
        p0 = P(*b["position"]) - ax * 3.0
        circ = rg.Circle(rg.Plane(p0, ax), HOLE_D[b["size"]] / 2.0)
        c = rg.Cylinder(circ, T_PLATE + 6.0)
        cyl.append(rg.Brep.CreateFromCylinder(c, True, True))
    return cyl


def build():
    A = a_plate()
    B = b_plate()
    W = web()
    part = one(rg.Brep.CreateBooleanUnion([A, B, W], TOL), "union")
    # trim at the bed
    big = rg.Box(rg.Plane.WorldXY, rg.Interval(-200, 400), rg.Interval(-300, 200),
                 rg.Interval(Z_LOW - 50, Z_BED)).ToBrep()
    part = one(rg.Brep.CreateBooleanDifference(part, big, TOL), "bed trim")
    part = one(rg.Brep.CreateBooleanDifference([part], holes(), TOL), "holes")
    part.MergeCoplanarFaces(TOL)
    solid_check(part, "final")
    vp = rg.VolumeMassProperties.Compute(part)
    log("faces %d, volume %.0f mm3" % (part.Faces.Count, vp.Volume))
    bb = part.GetBoundingBox(True)
    log("bbox", bb.Min, bb.Max)
    return [part]


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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(LOGLINES) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n" + "\n".join(LOGLINES) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
