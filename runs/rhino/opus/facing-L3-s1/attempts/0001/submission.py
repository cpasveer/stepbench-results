#! python3
"""Facing connector between mount A (below) and mount B (above), modelled in RhinoCommon.

Concept: a thin-walled tube, centred on B's axis, stands directly on A's face and
carries B's face on its top rim, so the load goes straight through the tube wall.
- Four flat lugs on the build plate (A side) take the M4 screws; heads sit on top.
- Three bosses at the top take the M5 screws from below. Each boss is a stadium
  that grows out of the tube wall; its underside is a 45 deg plane falling towards
  the tube, so it prints without support. The only downward face is the small
  ring a screw head bears on.
Printed A face down. All outer surfaces are vertical, horizontal-up or 45 deg.
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
msgs = []

# ---- parameters ----
ZA = FACES["A"]["origin"][2]          # A face (bottom), -94
ZB = FACES["B"]["origin"][2]          # B face (top), -25.5
CX, CY = FACES["B"]["origin"][0], FACES["B"]["origin"][1]   # tube axis = B axis
R_T = 29.5        # tube outer radius
W_T = 1.6         # tube wall
T_A = 4.0         # lug thickness (A grip)
R_LUG = 5.25      # A boss radius
D_A = 4.5         # M4 clearance
T_B = 5.0         # B boss grip
R_BOSS = 6.0      # B boss radius
D_B = 5.5         # M5 clearance
R_POCKET = 5.0    # head pocket under B bosses (head 4.25 + clearance)
DC = 11.5         # boss underside plane: depth = DC - offset (45 deg)


def P(x, y, z):
    return rg.Point3d(x, y, z)


def cyl(x, y, z0, z1, r):
    c = rg.Cylinder(rg.Circle(rg.Plane(P(x, y, z0), rg.Vector3d.ZAxis), r), z1 - z0)
    return c.ToBrep(True, True)


def prism(curve, h):
    srf = rg.Surface.CreateExtrusion(curve, rg.Vector3d(0, 0, h))
    b = srf.ToBrep().CapPlanarHoles(TOL)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def stadium(bx, by, z, r, a_in):
    """Footprint: semicircle radius r round (bx,by) on the far side from the axis,
    straight sides running back to distance a_in from the axis."""
    dx, dy = bx - CX, by - CY
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    vx, vy = -uy, ux
    s = a_in - d  # along u, relative to bolt (negative)
    p1 = P(bx + s * ux - r * vx, by + s * uy - r * vy, z)
    p2 = P(bx - r * vx, by - r * vy, z)
    pm = P(bx + r * ux, by + r * uy, z)
    p3 = P(bx + r * vx, by + r * vy, z)
    p4 = P(bx + s * ux + r * vx, by + s * uy + r * vy, z)
    pc = rg.PolyCurve()
    pc.Append(rg.Line(p1, p2))
    pc.Append(rg.Arc(p2, pm, p3))
    pc.Append(rg.Line(p3, p4))
    pc.Append(rg.Line(p4, p1))
    return pc, (ux, uy), (vx, vy), d


def union(breps):
    res = rg.Brep.CreateBooleanUnion(breps, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("union gave %s pieces" % (len(res) if res else None))
    return res[0]


def diff(a, cutters):
    res = rg.Brep.CreateBooleanDifference([a], cutters, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("difference gave %s pieces" % (len(res) if res else None))
    return res[0]


def build():
    a_in = R_T - W_T / 2.0
    tube = diff(cyl(CX, CY, ZA, ZB, R_T), [cyl(CX, CY, ZA - 1, ZB + 1, R_T - W_T)])
    parts = [tube]
    cutters = []
    for b in BOLTS:
        bx, by, bz = b["position"]
        if b["id"].startswith("A"):
            crv, u, v, d = stadium(bx, by, ZA, R_LUG, a_in)
            parts.append(prism(crv, T_A))
            cutters.append(cyl(bx, by, ZA - 1, ZA + T_A + 1, D_A / 2))
        else:
            crv, u, v, d = stadium(bx, by, ZB, R_BOSS, a_in)
            boss = prism(crv, -40.0)
            # cutter below the 45 deg plane depth = DC - off  (off along u from bolt)
            def wp(off, dep, lat):
                return P(bx + off * u[0] + lat * v[0], by + off * u[1] + lat * v[1], ZB - dep)
            pts = [wp(-40, DC + 40, -10), wp(20, DC - 20, -10), wp(20, 60, -10), wp(-40, 60, -10)]
            pts.append(pts[0])
            prof = rg.PolylineCurve(pts)
            srf = rg.Surface.CreateExtrusion(prof, rg.Vector3d(20 * v[0], 20 * v[1], 0))
            cut = srf.ToBrep().CapPlanarHoles(TOL)
            if cut.SolidOrientation == rg.BrepSolidOrientation.Inward:
                cut.Flip()
            boss = diff(boss, [cut])
            parts.append(boss)
            cutters.append(cyl(bx, by, ZB - T_B - 1, ZB + 1, D_B / 2))
            cutters.append(cyl(bx, by, ZB - 45, ZB - T_B, R_POCKET))
    body = union(parts)
    body = diff(body, cutters)
    body.MergeCoplanarFaces(TOL)
    msgs.append("solid=%s valid=%s faces=%d vol=%.1f" % (
        body.IsSolid, body.IsValid, body.Faces.Count, body.GetVolume()))
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
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(msgs) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(msgs))
    if os.path.exists(OUT):
        os.remove(OUT)
