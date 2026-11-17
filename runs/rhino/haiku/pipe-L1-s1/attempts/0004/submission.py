#! python3
"""Connecting bracket between pipe A (saddle clamp) and square mount B (plate), modelled in Rhino.

Saddle: 8 mm wall tube segment that seats on the 68 mm pipe (inner face r=34), 4 M5 bolts radial.
Plate: 30 x 30 x 8 mm on the 38 mm mount face, 4 M4 bolts on the 25 mm square.
Bridge: ruled loft from the saddle back to the plate, joining at a radial direction.
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
BOLT = {b["id"]: b for b in POSE["bolts"]}

TOL = 0.001
NOTES = []

PIPE_R = 34.0
SADDLE_T = 8.0          # wall of saddle, grip under M5 heads
SADDLE_HALF_L = 27.0    # half length of saddle along pipe axis
SADDLE_HALF_ANG = 50.0  # deg either side of the bolt centre line
M5_HOLE_R = 2.75        # 5.5 mm clearance
PLATE_HALF = 17.0
PLATE_T = 8.0           # grip under M4 heads
M4_HOLE_R = 2.25        # 4.5 mm clearance
BRIDGE_ROOT_R = 40.0
BRIDGE_ROOT_AX = 10.0
BRIDGE_ROOT_TAN = 5.0
BRIDGE_END_HU = 8.0   # bridge end on the plate, along u (away from the bolt heads)
BRIDGE_END_HV = 5.0


def V(x, y, z):
    return rg.Vector3d(x, y, z)


def dot(a, b):
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z


def unit(a):
    a = rg.Vector3d(a)
    a.Unitize()
    return a


def one(res, what):
    if res is None or len(res) != 1:
        n = 0 if res is None else len(res)
        raise RuntimeError("%s gave %d pieces" % (what, n))
    return res[0]


def union(a, b):
    return one(rg.Brep.CreateBooleanUnion([a, b], TOL), "union")


def minus(a, cutters):
    return one(rg.Brep.CreateBooleanDifference([a], cutters, TOL), "difference")


def cyl(origin, axis, r, length):
    return rg.Cylinder(rg.Circle(rg.Plane(origin, axis), r), length).ToBrep(True, True)


def prism(pts, axis, height):
    crv = rg.Polyline(pts + [pts[0]]).ToNurbsCurve()
    return rg.Extrusion.Create(crv, height, True).ToBrep()


def fit_axial(brep, axis, lo):
    """Translate brep along axis so that its lowest projection equals lo."""
    vals = [dot(rg.Vector3d(v.Location.X, v.Location.Y, v.Location.Z), axis) for v in brep.Vertices]
    shift = lo - min(vals)
    if abs(shift) > 1e-9:
        brep.Transform(rg.Transform.Translation(axis * shift))
    return brep


def build():
    P0 = rg.Point3d(21.0, 136.5, -7.0)
    d = unit(V(0.2487956429525063, 0.5900653629615055, 0.7680648380709507))
    # radial centre line of the saddle: bisector of the two bolt rows (bolt axes are radial)
    M = unit(V(*BOLT["A1"]["axis"]) + V(*BOLT["A3"]["axis"]))
    M = unit(M - d * dot(M, d))
    T = unit(rg.Vector3d.CrossProduct(d, M))

    # --- saddle on the pipe ---
    outer = cyl(P0 - d * 27.0, d, PIPE_R + SADDLE_T, 2.0 * SADDLE_HALF_L)
    inner = cyl(P0 - d * 30.0, d, PIPE_R, 60.0)
    ann = minus(outer, [inner])
    O = P0 - d * 30.0
    c = math.cos(math.radians(SADDLE_HALF_ANG))
    s = math.sin(math.radians(SADDLE_HALF_ANG))
    far = 100.0
    tri = [O,
           O + M * (far * c) + T * (far * s),
           O + M * (far * c) - T * (far * s)]
    sector = fit_axial(prism(tri, d, 60.0), d, dot(O, d))
    saddle = one(rg.Brep.CreateBooleanIntersection([ann], [sector], TOL), "saddle")

    # --- plate on the mounting face B (plate sits in front of the face, n >= 0) ---
    OB = rg.Point3d(19.549595153015062, 49.68566626427389, 52.2472790015625)
    u = unit(V(0.0, 0.0, 1.0))
    v = unit(V(0.9214414224885092, -0.3885173161164798, 0.0))
    n = unit(rg.Vector3d.CrossProduct(u, v))
    plate = rg.Box(rg.Plane(OB, u, v),
                   rg.Interval(-PLATE_HALF, PLATE_HALF),
                   rg.Interval(-PLATE_HALF, PLATE_HALF),
                   rg.Interval(0.0, PLATE_T)).ToBrep()

    # --- bridge: loft from saddle root (inside saddle) to plate centre (inside plate) ---
    R0 = P0 + M * BRIDGE_ROOT_R
    ax, tn = BRIDGE_ROOT_AX, BRIDGE_ROOT_TAN
    root = [R0 + d * ax + T * tn, R0 - d * ax + T * tn, R0 - d * ax - T * tn, R0 + d * ax - T * tn]
    Qc = OB + n * (PLATE_T / 2.0)
    hu, hv = BRIDGE_END_HU, BRIDGE_END_HV
    endp = [Qc + u * hu + v * hv, Qc - u * hu + v * hv, Qc - u * hu - v * hv, Qc + u * hu - v * hv]
    lofts = rg.Brep.CreateFromLoft(
        [rg.Polyline(root + [root[0]]).ToNurbsCurve(), rg.Polyline(endp + [endp[0]]).ToNurbsCurve()],
        rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Straight, False)
    bridge = one(lofts, "loft").CapPlanarHoles(TOL)

    parts = rg.Brep.CreateBooleanUnion([saddle, bridge], TOL)
    parts = rg.Brep.CreateBooleanUnion(list(parts) + [plate], TOL)
    body = one(parts, "body union")

    # --- clearance holes on every bolt axis (through) ---
    holes = []
    for bid in ["A1", "A2", "A3", "A4"]:
        p = rg.Point3d(*BOLT[bid]["position"])
        a = unit(V(*BOLT[bid]["axis"]))
        holes.append(cyl(p - a * 6.0, a, M5_HOLE_R, 22.0))
    for bid in ["B1", "B2", "B3", "B4"]:
        p = rg.Point3d(*BOLT[bid]["position"])
        a = unit(V(*BOLT[bid]["axis"]))
        holes.append(cyl(p - a * 3.0, a, M4_HOLE_R, 20.0))
    final = minus(body, holes)

    # --- self checks (informational) ---
    if not (final.IsSolid and final.IsValid):
        raise RuntimeError("result is not a valid closed solid")
    vol = rg.VolumeMassProperties.Compute(final).Volume
    NOTES.append("volume %.1f mm3" % vol)
    pipe = cyl(P0 - d * 130.0, d, PIPE_R, 260.0)
    ov = rg.Brep.CreateBooleanIntersection([final], [pipe], TOL)
    ovol = sum(rg.VolumeMassProperties.Compute(b).Volume for b in ov) if ov else 0.0
    NOTES.append("overlap with pipe %.4f mm3" % ovol)
    behind = min(dot(rg.Vector3d(x.Location.X - OB.X, x.Location.Y - OB.Y, x.Location.Z - OB.Z), n)
                 for x in final.Vertices)
    NOTES.append("lowest point behind mount face %.4f mm" % behind)
    return final


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    final = build()
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.Objects.AddBrep(final)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("\n".join(["ok: wrote output.step"] + NOTES) + "\n")


import math  # noqa: E402

try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
