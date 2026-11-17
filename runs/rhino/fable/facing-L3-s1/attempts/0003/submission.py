#! python3
"""facing-L3-s1 -- connecting bracket, modelled in Rhino (RhinoCommon).

Concept: a C-bracket that is printed lying on its back, without support.
  * flange A: a foot with a bar along the bed edge for the two near bolts and two
    splayed arms (steeper than 45 deg, so they print unsupported) to the two far bolts
  * flange B: a T-shaped head, a bar along the bed edge for B2/B3 and a stem to B1
  * the flanges are thick: only the bolt rings are clamped, so the arms and the stem
    carry the tilting load in bending; thickness is cheap when printed with sparse infill
  * a solid triangular spine joins them; its flat base lies on the printer bed,
    its sides lean inward (no overhang) and it stays clear of every bolt head and key.
All outlines are drawn in A's own frame (x, y along the square's sides), bed plane at y = YB.
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
BOLTS = POSE["resolved"]["bolts"]
PL_A = POSE["placements"]["A"]
TOL = 0.001

# ---- parameters -----------------------------------------------------------
T_A = 8.0          # flange A thickness (M4 grip 3..25)
T_B = 8.0          # flange B thickness (M5 grip 3.75..25)
D_A = 4.5          # M4 clearance hole
D_B = 5.5          # M5 clearance hole
YB = 34.0          # bed plane (A frame)
SP_X = -2.14       # spine centre line (in line with bolt B1)
SP_HALF = 18.5     # spine half width on the bed
SP_H = 30.0        # spine height off the bed

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

    # foot: bed bar + two arms from the spine to the far bolts
    HW = 4.25                       # half width of an arm (M4 head 7 + margin)
    E = 34.25                       # outer limit of the foot (face is +-36.5)
    c0 = (SP_X, 16.0)               # arms radiate from a point inside the spine

    def arm(bx, by, side):
        """outer/inner edge lines of the arm to bolt (bx, by); side=+1 right, -1 left."""
        dx, dy = bx - c0[0], by - c0[1]
        n = math.hypot(dx, dy)
        d = (dx / n, dy / n)
        no = (-d[1] * side, d[0] * side)          # outward normal (away from the other arm)
        po = (bx + HW * no[0], by + HW * no[1])
        pi = (bx - HW * no[0], by - HW * no[1])
        at_x = lambda p, x: (x, p[1] + (x - p[0]) * d[1] / d[0])
        at_y = lambda p, y: (p[0] + (y - p[1]) * d[0] / d[1], y)
        return po, pi, at_x, at_y

    far = {}
    for (lx, ly) in ((30.0, -30.0), (-30.0, -30.0)):
        far[lx > 0] = arm(lx, ly, 1 if lx > 0 else -1)
    poR, piR, axR, ayR = far[True]
    poL, piL, axL, ayL = far[False]
    XR, XL, YN = 13.5, -18.0, -4.0   # root fills beside the spine, flat bottom of the notch
    foot = [(-E, YB), (E, YB), (E, 28.0), (31.75, 25.5), (XR, 25.5), axR(poR, XR),
            axR(poR, E), (E, -31.75), (31.75, -E), ayR(piR, -E), ayR(piR, YN),
            ayL(piL, YN), ayL(piL, -E), (-31.75, -E), (-E, -31.75), axL(poL, -E),
            axL(poL, XL), (XL, 25.5), (-31.75, 25.5), (-E, 28.0)]
    log.append("foot " + " ".join("(%.2f,%.2f)" % p for p in foot))
    head = [(-35.0, YB), (24.6, YB), (29.72, 24.27), (28.0, 19.0), (12.0, 19.0), (3.11, 10.11),
            (3.11, -31.26), (0.11, -34.26), (-4.39, -34.26), (-7.39, -31.26), (-7.39, 7.3),
            (-16.0, 15.9), (-40.5, 15.9), (-42.46, 20.41)]
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
