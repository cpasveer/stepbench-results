#! python3
"""facing-L2-s1: turned "mushroom" bracket between two facing flanges.

Square 8 mm foot on A, round stem on B's axis with a flared base, and a funnel
head under B whose underside climbs at 50 degrees, so the part prints standing
on the foot without support. The M5 heads sit in radial slots cut up into the
funnel, open outward for the hex key; a central bore saves material and keeps sections even.
"""
import json, os, math, traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]
PA = POSE["placements"]["A"]
PB = POSE["placements"]["B"]
TOL = 0.001
msgs = []

O = PA["origin"]; R = PA["rotation"]
EX = (R[0][0], R[1][0]); EY = (R[0][1], R[1][1])   # A's local axes in world xy


def loc(p):
    dx, dy = p[0] - O[0], p[1] - O[1]
    return (dx * EX[0] + dy * EX[1], dx * EY[0] + dy * EY[1], p[2] - O[2])


def prism(pts, origin, xdir, ydir, height):
    """Closed prism: polygon pts (2D in plane origin/xdir/ydir) extruded along plane normal."""
    pl = rg.Plane(origin, xdir, ydir)
    p3 = [pl.PointAt(a, b) for a, b in pts]
    p3.append(p3[0])
    crv = rg.PolylineCurve(p3)
    face = rg.Brep.CreatePlanarBreps(crv, TOL)[0]
    path = rg.LineCurve(pl.Origin, pl.Origin + pl.Normal * height)
    b = face.Faces[0].CreateExtrusion(path, True)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    assert b.IsSolid
    return b


def revolve(profile, axis, dx=1.0, dy=0.0):
    """Revolve an open (r, z) polyline about a vertical axis and cap it to a solid."""
    parts = []
    for (r0, z0), (r1, z1) in zip(profile[:-1], profile[1:]):
        ln = rg.LineCurve(rg.Point3d(axis.From.X + r0 * dx, axis.From.Y + r0 * dy, z0),
                          rg.Point3d(axis.From.X + r1 * dx, axis.From.Y + r1 * dy, z1))
        parts.append(rg.Brep.CreateFromRevSurface(rg.RevSurface.Create(ln, axis), False, False))
    j = rg.Brep.JoinBreps(parts, TOL)
    assert j and len(j) == 1, "revolve join failed"
    b = j[0].CapPlanarHoles(TOL)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    assert b.IsSolid
    return b


def vcyl(u, v, z0, z1, r):
    circ = rg.Circle(rg.Plane(rg.Point3d(u, v, z0), rg.Vector3d.ZAxis), r)
    return rg.Cylinder(circ, z1 - z0).ToBrep(True, True)


def build():
    H = PB["origin"][2] - O[2]            # gap between the faces (68.5)
    TA = 8.0                              # flange on A = grip of the M4 bolts
    TB = 6.0                              # material under the M5 heads = grip
    HALF = 35.5                           # A seat: 1 mm inside the 73 mm face
    cu, cv, _ = loc(PB["origin"])         # B's axis in A's frame
    SLOPE = math.tan(math.radians(50.0))  # underside of the head: 40 deg overhang, prints unsupported

    # --- square foot on A (this face goes on the printer bed)
    sq = [(-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF)]
    foot = prism(sq, rg.Point3d(0, 0, 0), rg.Vector3d.XAxis, rg.Vector3d.YAxis, TA)

    # --- turned body on B's axis: base flare, stem, funnel head, top chamfer to the seat
    R_STEM, R_HEAD, R_SEAT, R_BASE = 16.0, 44.0, 39.5, 26.0
    z_seat = H - TB                       # level the M5 heads bear on
    z_rim = z_seat - 2.0                  # funnel ends in a short cylindrical rim
    z_neck = z_rim - (R_HEAD - R_STEM) * SLOPE
    z_base = TA + (R_BASE - R_STEM)
    assert z_neck > z_base + 2
    prof = [(R_BASE + 1.0, TA - 1.0), (R_STEM, z_base), (R_STEM, z_neck), (R_HEAD, z_rim),
            (R_HEAD, H - (R_HEAD - R_SEAT)), (R_SEAT, H)]
    axis = rg.Line(rg.Point3d(cu, cv, 0), rg.Point3d(cu, cv, H))
    body = revolve(prof, axis)

    u = rg.Brep.CreateBooleanUnion([foot, body], TOL)
    assert u and len(u) == 1, "union failed"
    body = u[0]

    # --- holes, head pockets under the B seat, central bore
    cutters = [vcyl(cu, cv, -1.0, H + 1.0, 9.0)]
    for b in BOLTS:
        lu, lv, lz = loc(b["position"])
        msgs.append("%s local %.2f %.2f %.2f" % (b["id"], lu, lv, lz))
        if lz < H / 2:
            assert math.hypot(lu - cu, lv - cv) - 4.0 > R_BASE + 1
            cutters.append(vcyl(lu, lv, -1.0, TA + 1.0, 2.25))
        else:
            assert math.hypot(lu - cu, lv - cv) + 4.25 < R_SEAT
            cutters.append(vcyl(lu, lv, z_seat - 1.0, H + 1.0, 2.75))
            # head + key slot, open to the outside: plane walls square to the funnel (no
            # feather edges), and the seat above it is a 9.6 mm bridge between the walls
            rb = math.hypot(lu - cu, lv - cv)
            dx, dy = (lu - cu) / rb, (lv - cv) / rb
            w, zlow = 4.8, 0.5 * (z_base + z_neck)
            rect = [(-w, -w), (R_HEAD - rb + 3.0, -w), (R_HEAD - rb + 3.0, w), (-w, w)]
            cutters.append(prism(rect, rg.Point3d(lu, lv, zlow), rg.Vector3d(dx, dy, 0), rg.Vector3d(-dy, dx, 0), z_seat - zlow))
    r = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    assert r and len(r) == 1, "difference failed"
    body = r[0]
    body.MergeCoplanarFaces(TOL)
    body.Compact()

    # --- to world
    wpl = rg.Plane(rg.Point3d(O[0], O[1], O[2]), rg.Vector3d(EX[0], EX[1], 0), rg.Vector3d(EY[0], EY[1], 0))
    body.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, wpl))
    assert body.IsSolid and body.IsValid
    vol = rg.VolumeMassProperties.Compute(body).Volume
    msgs.append("faces %d edges %d volume %.0f mm3" % (body.Faces.Count, body.Edges.Count, vol))
    for f in body.Faces:
        srf = f.UnderlyingSurface()
        kind = "plane" if srf.IsPlanar(TOL) else "cyl" if srf.IsCylinder(TOL) else "cone" if srf.IsCone(TOL) else "other"
        fb = f.GetBoundingBox(True)
        msgs.append("  face %d %s area %.1f z %.1f..%.1f" % (f.FaceIndex, kind, rg.AreaMassProperties.Compute(f).Area, fb.Min.Z, fb.Max.Z))
    bb = body.GetBoundingBox(True)
    msgs.append("bbox %s %s" % (bb.Min, bb.Max))
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(msgs) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
