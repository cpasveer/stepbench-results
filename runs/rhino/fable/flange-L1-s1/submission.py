#! python3
"""Bracket between round flange A and rectangular block B (task flange-L1-s1).

Design: a round foot plate on A, a rounded rectangular foot plate on B, joined by
a cruciform beam (vertical web + horizontal shelf) running square to B's face.
The beam passes between the bolt rows so every screw keeps free hex-key access.
"""
import json
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
BOLTS = RES["bolts"]
TOL = 0.001
MSG = []

# ---- design numbers (mm) ----
T_A = 8.0          # foot plate thickness on A  (= grip of the M5 screws)
T_B = 8.0          # foot plate thickness on B  (= grip of the M6 screws)
R_A = 36.0         # foot radius on A (face radius 37.25, 1.25 inside)
W_B, H_B = 43.5, 93.5   # foot on B (face 46 x 96, 1.25 inside)
R_CORNER = 4.0
D_M5, D_M6 = 5.5, 6.6   # clearance holes (ISO 273 medium)
S_BEAM = 2.5       # beam centre line, offset along B's width axis
T_WEB = 8.0
W_SHELF = 32.0
Z_SHELF = (1.5, 9.5)    # centred on flange axis height 5.5


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


def frame(name):
    pl = POSE["placements"][name]
    r = pl["rotation"]
    o = P(pl["origin"])
    x = rg.Vector3d(r[0][0], r[1][0], r[2][0])
    y = rg.Vector3d(r[0][1], r[1][1], r[2][1])
    n = rg.Vector3d(r[0][2], r[1][2], r[2][2])
    return o, x, y, n


OA, XA, YA, NA = frame("A")
OB, XB, YB, NB = frame("B")   # XB horizontal width axis, YB = world Z, NB outward normal
Z = rg.Vector3d(0, 0, 1)


def solid(b):
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    if not b.IsSolid:
        raise RuntimeError("not a closed solid")
    return b


def prism(pts, vec):
    """Closed prism: planar polygon pts swept along vec."""
    pl = rg.Polyline(list(pts) + [pts[0]]).ToNurbsCurve()
    return prism_crv(pl, vec)


def prism_crv(crv, vec):
    face = rg.Brep.CreatePlanarBreps(crv, TOL)[0].Faces[0]
    s = crv.PointAtStart
    path = rg.LineCurve(s, s + vec)
    return solid(face.CreateExtrusion(path, True))


def b_pt(s, d, z):
    """Point given in B's frame: s along width, d out of the face, z world height."""
    p = OB + XB * s + NB * d
    return rg.Point3d(p.X, p.Y, z)


def d_at_A(s, u):
    """d such that b_pt(s, d, .) lies at distance u in front of A's face."""
    p0 = OB + XB * s
    return (u - (p0 - OA) * NA) / (NB * NA)


def build():
    # --- foot on A: round plate ---
    circ = rg.Circle(rg.Plane(OA, XA, YA), R_A).ToNurbsCurve()
    foot_a = prism_crv(circ, NA * T_A)

    # --- foot on B: rounded rectangle ---
    rect = rg.Rectangle3d(rg.Plane(OB, XB, YB), rg.Interval(-W_B / 2, W_B / 2),
                          rg.Interval(-H_B / 2, H_B / 2)).ToNurbsCurve()
    rect = rg.Curve.CreateFilletCornersCurve(rect, R_CORNER, TOL, 0.01)
    foot_b = prism_crv(rect, NB * T_B)

    # --- beam, square to B's face, ends buried at mid thickness of both feet ---
    def plan_bar(half, z0, z1):
        s0, s1 = S_BEAM - half, S_BEAM + half
        pts = [b_pt(s0, T_B / 2, z0), b_pt(s1, T_B / 2, z0),
               b_pt(s1, d_at_A(s1, T_A / 2), z0), b_pt(s0, d_at_A(s0, T_A / 2), z0)]
        return prism(pts, Z * (z1 - z0))

    shelf = plan_bar(W_SHELF / 2, Z_SHELF[0], Z_SHELF[1])

    # web: tall at B (reaches towards both bolt rows), tapering to the flange
    d_b = T_B
    d_a = d_at_A(S_BEAM, T_A)
    top = lambda d: 26.0 + (34.0 - 26.0) * (d - d_b) / (d_a - d_b)
    bot = lambda d: -56.0 + (-23.0 + 56.0) * (d - d_b) / (d_a - d_b)
    da, db = -6.0, d_a + 20.0
    sa = S_BEAM - 15.0
    side = [b_pt(sa, da, bot(da)), b_pt(sa, db, bot(db)), b_pt(sa, db, top(db)), b_pt(sa, da, top(da))]
    wedge = prism(side, XB * 30.0)
    slab = plan_bar(T_WEB / 2, -80.0, 60.0)
    web = rg.Brep.CreateBooleanIntersection(slab, wedge, TOL)
    if web is None or len(web) != 1:
        raise RuntimeError("web intersection failed")
    web = solid(web[0])
    web.MergeCoplanarFaces(TOL)

    u = rg.Brep.CreateBooleanUnion([foot_a, foot_b, web, shelf], TOL)
    if u is None or len(u) != 1:
        raise RuntimeError("union failed: %s" % (None if u is None else len(u)))
    part = u[0]

    # --- clearance holes ---
    cutters = []
    for b in BOLTS:
        dia = D_M5 if b["size"] == "M5" else D_M6
        ax = V(b["axis"])
        base = P(b["position"]) - ax * 2.0
        cyl = rg.Cylinder(rg.Circle(rg.Plane(base, ax), dia / 2.0), 12.0)
        cutters.append(cyl.ToBrep(True, True))
    d = rg.Brep.CreateBooleanDifference([part], cutters, TOL)
    if d is None or len(d) != 1:
        raise RuntimeError("hole cut failed")
    part = d[0]
    part.MergeCoplanarFaces(TOL)
    part.Compact()
    solid(part)
    if not part.IsValid:
        raise RuntimeError("invalid brep")
    vol = rg.VolumeMassProperties.Compute(part).Volume
    emin = min(e.GetLength() for e in part.Edges)
    amin = min(rg.AreaMassProperties.Compute(f).Area for f in part.Faces)
    MSG.append("faces %d edges %d volume %.0f mm3 min edge %.3f min face %.3f"
               % (part.Faces.Count, part.Edges.Count, vol, emin, amin))
    return [part]


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
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
