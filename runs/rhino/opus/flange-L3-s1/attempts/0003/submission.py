#! python3
"""Flange-to-block connector, modelled in Rhino (RhinoCommon).

Concept
  * A plate: 4 mm disc on the round flange (seat kept 1 mm inside its outline),
    top cut flat + steep chamfers so it can stand on the print bed.
  * B plate: 5 mm plate on the block, extended up to the same bed plane; the
    extension is recessed 1 mm off the block face so the seat stays inside.
  * Body: a lofted beam between the plates' fronts (wide, low, in the band
    free of all bolt heads) with steep roofs rising to a narrow fin that runs
    up to the bed plane. Everything hangs from one flat face (z = Z_BED) so the
    part prints upside down without support.
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
MSG = []


def log(*a):
    MSG.append(" ".join(str(x) for x in a))


def V(x):
    return rg.Vector3d(x[0], x[1], x[2])


class Frame:
    """u horizontal in the face, z world up, n outward normal of the mounting face."""

    def __init__(self, part, uref):
        f = FACES[part]
        self.o = rg.Point3d(*f["origin"])
        self.n = V(f["normal"])
        self.n.Unitize()
        self.z = rg.Vector3d(0, 0, 1)
        u = rg.Vector3d.CrossProduct(self.z, self.n)
        u.Unitize()
        if u * uref < 0:
            u = -u
        self.u = u

    def p(self, u, zw, n):
        """point from (u, world z, n)."""
        q = self.o + self.u * u + self.n * n
        return rg.Point3d(q.X, q.Y, zw)

    def plane(self, n):
        return rg.Plane(self.o + self.n * n, self.u, self.z)


# uA chosen so that bolt A5 sits at +u; uB roughly parallel to uA
FA = Frame("A", rg.Vector3d(-0.6743, 0.7385, 0))
FB = Frame("B", FA.u)

T_A = 4.0      # plate thickness (grip) on A
T_B = 4.6      # plate thickness (grip) on B
Z_BED = 36.5   # print bed plane (part printed upside down)
ZA = FA.o.Z
ZB = FB.o.Z
R_SEAT_A = 74.5 / 2 - 1.0
B_HALF_U = 46 / 2 - 1.0
B_ZMIN = ZB - 96 / 2 + 1.0 + 0.5
B_ZSEAT_MAX = ZB + 96 / 2 - 1.0


def extrude_planar(curve, vec):
    """Extrude a closed planar curve segment by segment (no kinky faces), then cap."""
    segs = curve.DuplicateSegments()
    if not segs:
        segs = [curve]
    sides = [rg.Surface.CreateExtrusion(c, vec).ToBrep() for c in segs]
    j = rg.Brep.JoinBreps(sides, TOL)
    b = j[0].CapPlanarHoles(TOL)
    if b is None or not b.IsSolid:
        raise RuntimeError("cap failed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def polyline_curve(pts):
    pts = list(pts) + [pts[0]]
    return rg.PolylineCurve(pts)


def plate_A():
    pl = FA.plane(0.0)
    # seam of the circle at the top, where the chord cuts it away -> one arc
    circ = rg.ArcCurve(rg.Circle(rg.Plane(pl.Origin, pl.YAxis, -pl.XAxis), R_SEAT_A))
    # top cut flat at the bed, with steep (55 deg) chamfers
    zt = Z_BED - ZA
    w = math.sqrt(R_SEAT_A ** 2 - zt ** 2)
    k = math.tan(math.radians(55))
    big = 80.0
    poly2d = [(-big, -big), (big, -big), (big, zt - k * (big - w)), (w, zt), (-w, zt), (-big, zt - k * (big - w))]
    poly = polyline_curve([pl.PointAt(a, b) for a, b in poly2d])
    res = rg.Curve.CreateBooleanIntersection(circ, poly, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("plate A outline failed")
    return extrude_planar(res[0], FA.n * T_A)


def plate_B():
    pl = FB.plane(0.0)
    pts = [pl.PointAt(-B_HALF_U, B_ZMIN - ZB), pl.PointAt(B_HALF_U, B_ZMIN - ZB),
           pl.PointAt(B_HALF_U, Z_BED - ZB), pl.PointAt(-B_HALF_U, Z_BED - ZB)]
    b = extrude_planar(polyline_curve(pts), FB.n * T_B)
    # 1 mm recess on the back above the seat outline (seat stays inside the face)
    zs = B_ZSEAT_MAX - 0.3
    prof = [(-1.0, zs), (0.0, zs), (1.0, zs + 1.5), (1.0, Z_BED + 2), (-1.0, Z_BED + 2)]
    u0 = -B_HALF_U - 2
    cpts = [FB.p(u0, z, n) for n, z in prof]
    cut = extrude_planar(polyline_curve(cpts), FB.u * (2 * B_HALF_U + 4))
    # lightening window between the beam and the lower bolts: flat top, steep V bottom
    k = math.tan(math.radians(55))
    win = [(-10.0, -18.5), (10.0, -18.5), (10.0, -38.0), (0.0, -38.0 - 10.0 * k), (-10.0, -38.0)]
    wc = polyline_curve([FB.p(u, z, -1.0) for u, z in win])
    wcut = extrude_planar(wc, FB.n * (T_B + 2.0))
    r = rg.Brep.CreateBooleanDifference([b], [cut, wcut], TOL)
    return r[0]


# body sections: (u, world z) in each plate frame, same vertex order
SEC_A = [(-14.0, -14.5), (14.0, -14.5), (14.0, -1.0), (-1.0, 19.5),
         (-1.0, Z_BED), (-9.0, Z_BED), (-9.0, 6.0), (-14.0, -1.0)]
SEC_B = [(-13.0, -14.5), (4.0, -14.5), (4.0, -1.0), (-1.3, 6.5),
         (-1.3, Z_BED), (-8.7, Z_BED), (-8.7, 5.5), (-13.0, -1.0)]


def body():
    pa = [FA.p(u, z, T_A - 1.0) for u, z in SEC_A]
    pb = [FB.p(u, z, T_B - 1.0) for u, z in SEC_B]
    faces = []
    n = len(pa)
    for i in range(n):
        j = (i + 1) % n
        f = rg.Brep.CreateFromCornerPoints(pa[i], pa[j], pb[j], pb[i], TOL)
        faces.append(f)
    for pts in (pa, pb):
        cap = rg.Brep.CreatePlanarBreps(polyline_curve(pts), TOL)
        faces.append(cap[0])
    j = rg.Brep.JoinBreps(faces, TOL)
    b = j[0]
    if not b.IsSolid:
        raise RuntimeError("body not closed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def bolt_frame(bolt):
    p = rg.Point3d(*bolt["position"])
    a = V(bolt["axis"])
    a.Unitize()
    return p, a


def cyl(p, a, r, n0, n1):
    pl = rg.Plane(p + a * n0, a)
    c = rg.Cylinder(rg.Circle(pl, r), n1 - n0)
    return c.ToBrep(True, True)


def holes():
    out = []
    for b in BOLTS:
        p, a = bolt_frame(b)
        d = 5.5 if b["size"] == "M5" else 6.6
        out.append(cyl(p, a, d / 2, -2.0, 8.0))
    return out


def build():
    parts = [plate_A(), plate_B(), body()]
    u = rg.Brep.CreateBooleanUnion(parts, TOL)
    if not u or len(u) != 1:
        raise RuntimeError("union gave %s pieces" % (len(u) if u else 0))
    s = u[0]
    d = rg.Brep.CreateBooleanDifference([s], holes(), TOL)
    if not d or len(d) != 1:
        raise RuntimeError("hole cut failed")
    s = d[0]
    s.MergeCoplanarFaces(0.01, math.radians(0.5))
    merged = True
    while merged:
        merged = False
        bed = [f.FaceIndex for f in s.Faces if _is_bed(f)]
        for i in range(len(bed)):
            for j in range(i + 1, len(bed)):
                if s.MergeCoplanarFaces(bed[i], bed[j], 0.01, math.radians(0.5)):
                    merged = True
                    break
            if merged:
                break
    log("bed faces", len([f for f in s.Faces if _is_bed(f)]))
    for f in s.Faces:
        am = rg.AreaMassProperties.Compute(f.DuplicateFace(False))
        pass  # log("  face", f.FaceIndex, f.UnderlyingSurface().GetType().Name, "%.1f" % am.Area, "loops", f.Loops.Count)
    return s


def _is_bed(f):
    ok, pl = f.TryGetPlane(0.01)
    return ok and abs(abs(pl.Normal.Z) - 1) < 1e-6 and abs(pl.Origin.Z - Z_BED) < 0.01


# ---------------------------------------------------------------- self checks
def vol_inter(a, b):
    r = rg.Brep.CreateBooleanIntersection(a, b, TOL)
    if not r:
        return 0.0
    return sum(abs(x.GetVolume()) for x in r)


def self_check(s):
    log("valid", s.IsValid, "solid", s.IsSolid, "faces", s.Faces.Count, "volume %.0f" % s.GetVolume())
    for b in BOLTS:
        p, a = bolt_frame(b)
        t = T_A if b["id"][0] == "A" else T_B
        hr, hh = b["head_diameter"] / 2, b["head_height"]
        head = cyl(p, a, hr + 0.5, t + 0.01, t + hh + 1.0)
        tool = cyl(p, a, hr, t + hh, t + hh + 30.0)
        vh, vt = vol_inter(s, head), vol_inter(s, tool)
        # key long leg: perpendicular to axis at ~20 mm above head top, radial-outward, +-30 deg sweep
        fr = FA if b["id"][0] == "A" else FB
        c = fr.o + fr.n * (p - fr.o) * fr.n
        rad = p - fr.o
        rad = rad - fr.n * (rad * fr.n)
        rad.Unitize()
        hit = []
        for ang in (-30, -15, 0, 15, 30):
            d = rg.Vector3d(rad)
            d.Rotate(math.radians(ang), a)
            leg_c = p + a * (t + hh + 20.5)
            leg = cyl(leg_c, d, 2.3, 0.0, 70.0)
            if vol_inter(s, leg) > 0.01:
                hit.append(ang)
        log(b["id"], "head %.3f tool %.3f keyhits %s" % (vh, vt, hit))
    # overhang (printed upside down: build direction = -z)
    mp = rg.MeshingParameters.QualityRenderMesh
    ms = rg.Mesh.CreateFromBrep(s, mp)
    m = rg.Mesh()
    for x in ms:
        m.Append(x)
    m.FaceNormals.ComputeFaceNormals()
    bad = 0.0
    for i in range(m.Faces.Count):
        nz = m.FaceNormals[i].Z
        f = m.Faces[i]
        pts = [m.Vertices[f.A], m.Vertices[f.B], m.Vertices[f.C]]
        zc = sum(q.Z for q in pts) / 3
        cen = rg.Point3d((pts[0].X + pts[1].X + pts[2].X) / 3, (pts[0].Y + pts[1].Y + pts[2].Y) / 3, zc)
        inhole = False
        for b in BOLTS:
            p, a = bolt_frame(b)
            w = cen - p
            w = w - a * (w * a)
            if w.Length < 3.6:
                inhole = True
        if nz > 0.7072 and abs(zc - Z_BED) > 0.01 and not inhole:
            log("  overhang face at", "%.1f %.1f %.1f nz %.2f" % (cen.X, cen.Y, cen.Z, nz))
            a = rg.Vector3d.CrossProduct(rg.Vector3d(pts[1] - pts[0]), rg.Vector3d(pts[2] - pts[0])).Length / 2
            if f.IsQuad:
                a *= 2
            bad += a
    log("overhang area (excl. holes) %.1f" % bad)


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        s = build()
        self_check(s)
        doc.Objects.AddBrep(s)
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
