#! python3
"""facing-L1-s1: spool-shaped adapter between a square M4 mount (A, below) and a round M5 mount (B, above).

A square foot flange seats on A, a round flange seats on B, and a hollow round
column with generous root fillets joins them.  The column stands midway between
the two (slightly offset) mount centres and is slim enough that every screw
head is in the open, so each hex key can swing freely outward.
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
BOLTS = RES["bolts"]
TOL = 0.001

# ---- design numbers (mm) ----
T_A = 7.0          # foot flange thickness (M4 grip)
T_B = 7.0          # top flange thickness (M5 grip)
SETBACK = 1.25     # flange edge inside the mounting face outline
R_COL = 18.0       # column outer radius
R_BORE = 10.0      # through bore (saves material, passes cables)
R_FIL = 6.0        # root fillets column -> flanges
D_HOLE = {"M4": 4.5, "M5": 5.5}

notes = []


def P(x, y, z):
    return rg.Point3d(x, y, z)


def solid_from_faces(breps):
    j = rg.Brep.JoinBreps(breps, TOL)
    if j is None or len(j) != 1:
        raise RuntimeError("join gave %s pieces" % (None if j is None else len(j)))
    b = j[0]
    if not b.IsSolid:
        b = b.CapPlanarHoles(TOL)
    if b is None or not b.IsSolid:
        raise RuntimeError("not a solid")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def prism(segments, z0, z1):
    vec = rg.Vector3d(0, 0, z1 - z0)
    sides = []
    for s in segments:
        c = s.DuplicateCurve()
        c.Translate(rg.Vector3d(0, 0, z0))
        sides.append(rg.Surface.CreateExtrusion(c, vec).ToBrep())
    return solid_from_faces(sides)


def cyl(cx, cy, r, z0, z1):
    circ = rg.Circle(rg.Plane(P(cx, cy, z0), rg.Vector3d.ZAxis), r)
    return rg.Cylinder(circ, z1 - z0).ToBrep(True, True)


def build():
    pl = POSE["placements"]
    oa, ra = pl["A"]["origin"], pl["A"]["rotation"]
    ob = pl["B"]["origin"]
    za, zb = oa[2], ob[2]                      # seat planes: A faces up, B faces down
    fa = TASK["fixed_parts"][0]["interface"]
    fb = TASK["fixed_parts"][1]["interface"]

    # --- foot flange on A: square, corners rounded concentric with the bolts ---
    half = fa["face_size"][0] / 2.0 - SETBACK
    pitch2 = fa["params"]["pitch"] / 2.0
    rc = half - pitch2                         # corner radius centred on each bolt
    ux, uy = ra[0][0], ra[1][0]                # local x in world
    vx, vy = ra[0][1], ra[1][1]                # local y in world

    def w(lx, ly):
        return P(oa[0] + ux * lx + vx * ly, oa[1] + uy * lx + vy * ly, 0.0)

    segs = []
    k = math.sqrt(0.5)
    for i in range(4):
        a = math.radians(90 * i)
        c, s = math.cos(a), math.sin(a)

        def rot(lx, ly):
            return w(c * lx - s * ly, s * lx + c * ly)
        # straight side along +x edge, then the corner at (+,+)
        segs.append(rg.LineCurve(rot(half, -pitch2), rot(half, pitch2)))
        segs.append(rg.ArcCurve(rg.Arc(rot(half, pitch2), rot(pitch2 + rc * k, pitch2 + rc * k), rot(pitch2, half))))
    foot = prism(segs, za, za + T_A)

    # --- top flange on B: round ---
    r_top = fb["face_size"][0] / 2.0 - SETBACK
    top = cyl(ob[0], ob[1], r_top, zb - T_B, zb)

    # --- column with root fillets, revolved about an axis midway between the mounts ---
    cx, cy = (oa[0] + ob[0]) / 2.0, (oa[1] + ob[1]) / 2.0
    rf = R_COL + R_FIL
    z1, z2 = za + T_A, zb - T_B

    def q(r, z):
        return P(cx + r, cy, z)
    kk = R_FIL * (1 - k)
    prof = [
        rg.LineCurve(q(0, za), q(rf, za)),
        rg.LineCurve(q(rf, za), q(rf, z1)),
        rg.ArcCurve(rg.Arc(q(rf, z1), q(R_COL + kk, z1 + kk), q(R_COL, z1 + R_FIL))),
        rg.LineCurve(q(R_COL, z1 + R_FIL), q(R_COL, z2 - R_FIL)),
        rg.ArcCurve(rg.Arc(q(R_COL, z2 - R_FIL), q(R_COL + kk, z2 - kk), q(rf, z2))),
        rg.LineCurve(q(rf, z2), q(rf, zb)),
        rg.LineCurve(q(rf, zb), q(0, zb)),
    ]
    axis = rg.Line(P(cx, cy, za), P(cx, cy, zb))
    col = solid_from_faces([rg.RevSurface.Create(c, axis).ToBrep() for c in prof])

    u = rg.Brep.CreateBooleanUnion([foot, col, top], TOL)
    if u is None or len(u) != 1:
        raise RuntimeError("union failed")
    body = u[0]

    # --- bore and bolt holes ---
    cutters = [cyl(cx, cy, R_BORE, za - 5, zb + 5)]
    for b in BOLTS:
        p, ax = b["position"], b["axis"]
        d = D_HOLE[b["size"]]
        if ax[2] > 0:
            cutters.append(cyl(p[0], p[1], d / 2.0, za - 5, za + T_A + 5))
        else:
            cutters.append(cyl(p[0], p[1], d / 2.0, zb - T_B - 5, zb + 5))
    for cu in cutters:
        r = rg.Brep.CreateBooleanDifference(body, cu, TOL)
        if r is None or len(r) != 1:
            raise RuntimeError("cut failed")
        body = r[0]
    body.MergeCoplanarFaces(TOL)
    body.Compact()
    ok, msg = body.IsValidWithLog()
    if not (ok and body.IsSolid):
        raise RuntimeError("invalid body: " + msg)
    vol = rg.VolumeMassProperties.Compute(body).Volume
    bb = body.GetBoundingBox(True)
    notes.append("faces %d edges %d volume %.0f mm3" % (body.Faces.Count, body.Edges.Count, vol))
    notes.append("bbox %s .. %s" % (bb.Min, bb.Max))
    notes.append("min edge %.3f" % min(e.GetLength() for e in body.Edges))
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(notes) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
