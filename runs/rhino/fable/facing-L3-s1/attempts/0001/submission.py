#! python3
"""facing-L3-s1 -- connecting bracket, modelled in Rhino (RhinoCommon).

Concept: a C-bracket that is printed lying on its back, without support.
  * flange A (3.2 mm): a U-shaped foot, one bar along the bed edge and two legs to the far bolts
  * flange B (4.0 mm): a T-shaped head, a bar along the bed edge for B2/B3 and a stem to B1
  * a solid triangular spine joins them; its flat base lies on the printer bed,
    its sides lean inward (no overhang) and it stays clear of every bolt head and key.
All outlines are drawn in A's own frame (x, y along the square's sides), bed plane at y = YB.
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
BOLTS = POSE["resolved"]["bolts"]
PL_A = POSE["placements"]["A"]
TOL = 0.001

# ---- parameters -----------------------------------------------------------
T_A = 3.2          # flange A thickness (M4 grip >= 3)
T_B = 4.0          # flange B thickness (M5 grip >= 3.75)
D_A = 4.5          # M4 clearance hole
D_B = 5.5          # M5 clearance hole
YB = 34.0          # bed plane (A frame)
SP_X = -2.14       # spine centre line (in line with bolt B1)
SP_HALF = 17.0     # spine half width on the bed
SP_H = 24.0        # spine height off the bed

O = PL_A["origin"]
R = PL_A["rotation"]
EX = (R[0][0], R[1][0])
EY = (R[0][1], R[1][1])
Z_A = O[2]
Z_B = POSE["placements"]["B"]["origin"][2]


def W(x, y, z):
    return rg.Point3d(O[0] + x * EX[0] + y * EY[0], O[1] + x * EX[1] + y * EY[1], z)


def prism(pts, z0, z1, holes=()):
    """Closed solid: polygon pts (A frame) from z0 to z1, with round holes (world xy, dia)."""
    poly = rg.Polyline([W(x, y, z0) for x, y in pts] + [W(pts[0][0], pts[0][1], z0)])
    curves = [poly.ToNurbsCurve()]
    for (hx, hy, d) in holes:
        c = rg.Circle(rg.Plane(rg.Point3d(hx, hy, z0), rg.Vector3d.ZAxis), d / 2.0)
        curves.append(c.ToNurbsCurve())
    faces = rg.Brep.CreatePlanarBreps(curves, TOL)
    if not faces or len(faces) != 1:
        raise RuntimeError("planar face failed")
    path = rg.LineCurve(rg.Point3d(0, 0, 0), rg.Point3d(0, 0, z1 - z0))
    solid = faces[0].Faces[0].CreateExtrusion(path, True)
    if solid is None or not solid.IsSolid:
        raise RuntimeError("extrusion failed")
    if solid.SolidOrientation == rg.BrepSolidOrientation.Inward:
        solid.Flip()
    return solid


def build(log):
    ha = [(b["position"][0], b["position"][1], D_A) for b in BOLTS if b["id"].startswith("A")]
    hb = [(b["position"][0], b["position"][1], D_B) for b in BOLTS if b["id"].startswith("B")]

    foot = [(-34.25, YB), (34.25, YB), (34.25, -31.75), (31.75, -34.25), (28.25, -34.25),
            (25.75, -31.75), (25.75, 22.5), (22.75, 25.5), (-22.75, 25.5), (-25.75, 22.5),
            (-25.75, -31.75), (-28.25, -34.25), (-31.75, -34.25), (-34.25, -31.75)]
    head = [(-35.0, YB), (24.6, YB), (29.72, 24.27), (28.0, 19.0), (6.11, 19.0), (3.11, 16.0),
            (3.11, -31.26), (0.11, -34.26), (-4.39, -34.26), (-7.39, -31.26), (-7.39, 12.9),
            (-10.39, 15.9), (-40.5, 15.9), (-42.46, 20.41)]
    spine = [(SP_X - SP_HALF, YB), (SP_X + SP_HALF, YB), (SP_X, YB - SP_H)]

    a = prism(foot, Z_A, Z_A + T_A, ha)
    b = prism(head, Z_B - T_B, Z_B, hb)
    s = prism(spine, Z_A, Z_B)
    u = rg.Brep.CreateBooleanUnion([a, b, s], TOL)
    if not u or len(u) != 1:
        raise RuntimeError("union gave %s pieces" % (len(u) if u else 0))
    part = u[0]
    part.MergeCoplanarFaces(TOL)
    part.Compact()
    vol = rg.VolumeMassProperties.Compute(part).Volume
    bb = part.GetBoundingBox(True)
    log.append("solid=%s valid=%s faces=%d edges=%d volume=%.1f" % (
        part.IsSolid, part.IsValid, part.Faces.Count, part.Edges.Count, vol))
    log.append("bbox %s .. %s" % (bb.Min, bb.Max))
    log.append("min edge %.3f" % min(e.GetLength() for e in part.Edges))
    if not (part.IsSolid and part.IsValid):
        raise RuntimeError("part is not a valid closed solid")
    return [part]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    log = []
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build(log):
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(log) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
