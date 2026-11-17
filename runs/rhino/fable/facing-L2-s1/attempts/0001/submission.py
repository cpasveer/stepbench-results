#! python3
"""facing-L2-s1: box-section bracket between two facing flanges.

Two 8 mm flanges (square on A, polygon inside B's round face) joined by two
6 mm webs with 45-degree root chamfers: a closed box section, extruded along
A's local -v axis so it prints lying on the side v = +35.5 with no overhangs
(all walls vertical, flange outlines climb at 45 degrees from the bed edge).
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


def build():
    H = PB["origin"][2] - O[2]            # gap between the faces (68.5)
    T = 8.0                               # flange thickness = grip
    HALF = 35.5                           # A seat half-size (73/2 - 1.5 ... 1 mm inside)
    BED = HALF                            # print bed plane v = +35.5
    cu, cv, _ = loc(PB["origin"])
    RB = 84.5 / 2 - 1.0                   # seat limit radius on B

    # --- flange on A: square, two chamfered corners on the far (top-in-print) side
    L = 3.0
    a_pts = [(-HALF, BED), (HALF, BED), (HALF, -HALF + L), (HALF - L, -HALF),
             (-HALF + L, -HALF), (-HALF, -HALF + L)]
    fa = prism(a_pts, rg.Point3d(0, 0, 0), rg.Vector3d.XAxis, rg.Vector3d.YAxis, T)

    # --- flange on B: polygon inside the round face, flat on the bed, 45 deg shoulders
    qb = BED - cv
    s, qs = 36.8, 18.5
    c = s + qs
    fl = c - qb
    tw, qt = 11.0, -39.6
    b_rel = [(-fl, qb), (fl, qb), (s, qs), (s, -qs), (tw, qt), (-tw, qt), (-s, -qs), (-s, qs)]
    for p, q in b_rel:
        assert math.hypot(p, q) < RB - 0.02, (p, q)
    b_pts = [(p + cu, q + cv) for p, q in b_rel]
    fb = prism(b_pts, rg.Point3d(0, 0, H - T), rg.Vector3d.XAxis, rg.Vector3d.YAxis, T)

    # --- two webs with chamfered roots, extruded along v (the print direction)
    WT, CH, V_END = 6.0, 3.0, -20.0
    webs = []
    for uc in (-16.0, 10.0):
        ul, ur = uc - WT / 2, uc + WT / 2
        z0, z1 = T, H - T
        prof = [(ul - CH, z0 - 1), (ur + CH, z0 - 1), (ur + CH, z0), (ur, z0 + CH),
                (ur, z1 - CH), (ur + CH, z1), (ur + CH, z1 + 1), (ul - CH, z1 + 1),
                (ul - CH, z1), (ul, z1 - CH), (ul, z0 + CH), (ul - CH, z0)]
        # plane x = u, y = z, normal = -v ; start on the bed, extrude to V_END
        webs.append(prism(prof, rg.Point3d(0, BED, 0), rg.Vector3d.XAxis, rg.Vector3d.ZAxis, BED - V_END))

    u = rg.Brep.CreateBooleanUnion([fa, fb] + webs, TOL)
    assert u and len(u) == 1, "union failed"
    body = u[0]

    # --- bolt holes
    cutters = []
    for b in BOLTS:
        lu, lv, lz = loc(b["position"])
        d = 4.5 if b["size"] == "M4" else 5.5
        zc = -1.0 if lz < H / 2 else H - T - 1.0
        if lz < H / 2:
            assert abs(lu) + 3.5 < HALF and abs(lv) + 3.5 < HALF
        circ = rg.Circle(rg.Plane(rg.Point3d(lu, lv, zc), rg.Vector3d.ZAxis), d / 2)
        cutters.append(rg.Cylinder(circ, T + 2.0).ToBrep(True, True))
        msgs.append("%s local %.2f %.2f %.2f" % (b["id"], lu, lv, lz))
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
