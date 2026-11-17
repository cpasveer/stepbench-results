#! python3
"""Connecting strut between round flange A and rectangular block B, modelled in Rhino.

Design: a round flange plate on A (6x M5 counterbore-free clearance holes), a
rounded rectangular plate on B (4x M6), joined by one round strut that runs
through the free gap between the two bolt patterns, so every socket head can
be reached and turned with a hex key swung outwards. Strut joints are filleted.
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
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TOL = 0.001
MSG = []

# ---- design parameters ----
T_A = 8.0            # plate thickness on A (grip of M5)
T_B = 8.0            # plate thickness on B (grip of M6)
R_PLATE_A = 35.75    # A face is 74.5 dia -> 1.5 mm inside the outline
W_B, H_B, RC_B = 43.0, 93.0, 6.0   # B face is 46 x 96
HOLE = {"M5": 5.5, "M6": 6.6}
R_STRUT = 14.0
FILLET = 3.0
# strut end points: A frame (lateral, z) and B frame (lateral, z), set from scratch/opt.py
STRUT_A = (-5.0, 2.0)
STRUT_B = (1.0, 15.0)


def V(p):
    return rg.Vector3d(p[0], p[1], p[2])


def P(p):
    return rg.Point3d(p[0], p[1], p[2])


def frame(part):
    f = FACES[part]
    o = P(f["origin"])
    n = V(f["normal"])
    n.Unitize()
    z = rg.Vector3d(0, 0, 1)
    u = rg.Vector3d.CrossProduct(z, n)
    u.Unitize()
    return o, u, z, n


def solid(brep, name):
    if brep is None or not brep.IsSolid:
        raise RuntimeError("not a closed solid: " + name)
    if brep.SolidOrientation == rg.BrepSolidOrientation.Inward:
        brep.Flip()
    return brep


def extrude(curve, vec, name):
    srf = rg.Surface.CreateExtrusion(curve, vec)
    b = srf.ToBrep().CapPlanarHoles(TOL)
    return solid(b, name)


def plate_A():
    o, u, z, n = frame("A")
    c = rg.Circle(rg.Plane(o, u, z), R_PLATE_A).ToNurbsCurve()
    return extrude(c, n * T_A, "plate A")


def plate_B():
    o, u, z, n = frame("B")
    pl = rg.Plane(o, u, z)
    rect = rg.Rectangle3d(pl, rg.Interval(-W_B / 2, W_B / 2), rg.Interval(-H_B / 2, H_B / 2))
    crv = rect.ToNurbsCurve()
    crv = rg.Curve.CreateFilletCornersCurve(crv, RC_B, TOL, 0.01)
    return extrude(crv, n * T_B, "plate B")


def strut():
    oA, uA, zA, nA = frame("A")
    oB, uB, zB, nB = frame("B")
    pa = oA + uA * STRUT_A[0] + zA * STRUT_A[1] + nA * (T_A * 0.5)
    pb = oB + uB * STRUT_B[0] + zB * STRUT_B[1] + nB * (T_B * 0.5)
    ax = pb - pa
    L = ax.Length
    ax.Unitize()
    cyl = rg.Cylinder(rg.Circle(rg.Plane(pa, ax), R_STRUT), L)
    MSG.append("strut length %.1f, angle to nA %.1f deg, to -nB %.1f deg" % (
        L, rg.Vector3d.VectorAngle(ax, nA) * 57.2958, rg.Vector3d.VectorAngle(ax, -nB) * 57.2958))
    return solid(cyl.ToBrep(True, True), "strut")


def union(breps):
    r = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("union gave %s pieces" % (None if r is None else len(r)))
    return r[0]


def holes(body):
    cutters = []
    for b in BOLTS:
        a = V(b["axis"])
        a.Unitize()
        t = T_A if b["id"].startswith("A") else T_B
        p = P(b["position"]) - a * 1.0
        cyl = rg.Cylinder(rg.Circle(rg.Plane(p, a), HOLE[b["size"]] / 2), t + 2.0)
        cutters.append(cyl.ToBrep(True, True))
    r = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    if not r or len(r) != 1:
        raise RuntimeError("hole cut gave %s pieces" % (None if r is None else len(r)))
    return r[0]


def fillet_joints(body, strut_brep):
    """Fillet the edges where the strut meets the two plates."""
    oA, uA, zA, nA = frame("A")
    oB, uB, zB, nB = frame("B")
    planeA = rg.Plane(oA + nA * T_A, nA)
    planeB = rg.Plane(oB + nB * T_B, nB)
    ids = []
    for e in body.Edges:
        mid = e.PointAt(e.Domain.Mid)
        on_plate = abs(planeA.DistanceTo(mid)) < 0.01 or abs(planeB.DistanceTo(mid)) < 0.01
        if not on_plate:
            continue
        # edge of the joint lies on the strut surface
        cp = strut_brep.ClosestPoint(mid)
        if cp.DistanceTo(mid) < 0.01:
            ids.append(e.EdgeIndex)
    MSG.append("joint edges: %d" % len(ids))
    if not ids:
        return body
    rad = [FILLET] * len(ids)
    r = rg.Brep.CreateFilletEdges(body, ids, rad, rad, rg.BlendType.Fillet,
                                  rg.RailType.RollingBall, TOL)
    if not r or len(r) != 1 or not r[0].IsSolid:
        MSG.append("fillet failed; keeping sharp joints")
        return body
    return r[0]


def build():
    s = strut()
    body = union([plate_A(), plate_B(), s])
    body = fillet_joints(body, s)
    body = holes(body)
    body.MergeCoplanarFaces(TOL)
    body = solid(body, "final")
    MSG.append("faces %d, volume %.0f mm3" % (body.Faces.Count, body.GetVolume()))
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(MSG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + "\n".join(MSG) + "\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
