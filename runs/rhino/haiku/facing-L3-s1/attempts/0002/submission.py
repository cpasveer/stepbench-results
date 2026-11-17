#! python3
"""Connecting part between fixed mount A (below, square, 4x M4) and fixed mount B (above, round, 3x M5).

Design
- A hollow tube (outer 40 mm, wall 1.2 mm) stands between the two mounting faces
  and carries the load. Its ends rest on face A and face B.
- Face A end: four flat arms run from the tube to four bosses on the A holes.
  Each boss is the seat for one M4 head; grips differ per bolt so that every
  bearing face is a plane of its own.
- Face B end: three gussets run from the tube to three bosses on the B holes.
  Their top is flush with face B; underneath they rise at 47 deg towards the
  tube so that the part prints without support.
- Keys and tools: the head zones and the 60-degree key swing point outwards from
  the tube, and no other part of the solid is within reach of a key or tool.

Only output.step is graded. All numbers come from task.json.
Run inside Rhino:   sh run_rhino.sh
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
BOLTS = POSE["bolts"]

# Mounting faces (world Z). Part A: normal +z, part sits above it.
# Part B: normal -z, part sits below it.
ZA = -94.0
ZB = -25.5

# Tube between the faces, centred between the two mount centres.
CA = (-126.0, 127.5)
CB = (-124.18164228311065, 135.11206773573574)
PX = 0.5 * (CA[0] + CB[0])
PY = 0.5 * (CA[1] + CB[1])

TUBE_R = 20.0
TUBE_WALL = 1.2
ARM_START = TUBE_R - 0.6          # arms start inside the tube wall

# Grip per bolt (thickness under the head, mm). Distinct values, so that each
# bearing face is a plane of its own.
GRIP = {"A1": 3.5, "A2": 4.0, "A3": 3.2, "A4": 4.5,
        "B1": 6.5, "B2": 7.0, "B3": 6.0}

A_BOSS_R = 4.0
A_HOLE_D = 4.5
A_ARM_W = 7.0

B_BOSS_R = 5.0
B_HOLE_D = 5.5
B_ARM_W = 8.0
B_FLAT = 5.5                      # flat underside around each B boss
SLOPE_B = math.tan(math.radians(47.0))


def cylinder(cx, cy, z0, h, r):
    circ = rg.Circle(rg.Plane(rg.Point3d(cx, cy, z0), rg.Vector3d.ZAxis), r)
    return rg.Cylinder(circ, h).ToBrep(True, True)


def arm(px, py, tx, ty, width, z0, h):
    """Horizontal rectangular arm from inside the tube wall towards (tx, ty)."""
    dx, dy = tx - px, ty - py
    length = math.hypot(dx, dy)
    ax, ay = dx / length, dy / length
    pl = rg.Plane(rg.Point3d(px, py, z0), rg.Vector3d(ax, ay, 0.0), rg.Vector3d(-ay, ax, 0.0))
    box = rg.Box(pl, rg.Interval(ARM_START, length),
                 rg.Interval(-0.5 * width, 0.5 * width),
                 rg.Interval(0.0, h))
    return box.ToBrep()


def gusset(px, py, tx, ty, width, z_top, z_boss_bottom):
    """Gusset from the tube to a B boss: flat top on face B, flat under the head
    zone, then rising at 47 deg towards the tube."""
    dx, dy = tx - px, ty - py
    lb = math.hypot(dx, dy)
    ax, ay = dx / lb, dy / lb
    s_flat = lb - B_FLAT
    s0 = ARM_START
    z_s0 = z_boss_bottom - (s_flat - s0) * SLOPE_B
    pl = rg.Plane(rg.Point3d(px, py, 0.0), rg.Vector3d(ax, ay, 0.0), rg.Vector3d.ZAxis)
    prof = [(s0, z_top), (lb, z_top), (lb, z_boss_bottom),
            (s_flat, z_boss_bottom), (s0, z_s0)]
    pts = [rg.Point3d(px, py, 0.0) + pl.XAxis * s + pl.YAxis * z for s, z in prof]
    pts.append(pts[0])
    crv = rg.Polyline(pts).ToNurbsCurve()
    lat = rg.Vector3d(-ay, ax, 0.0)
    crv.Transform(rg.Transform.Translation(lat * (-0.5 * width)))
    ext = rg.Extrusion.Create(crv, width, True).ToBrep()
    # centre the prism laterally on the arm axis (its centroid is on the mid-plane)
    c = rg.VolumeMassProperties.Compute(ext).Centroid
    shift = -(lat.X * (c.X - px) + lat.Y * (c.Y - py))
    ext.Transform(rg.Transform.Translation(lat * shift))
    return ext


def bool_union(parts):
    res = rg.Brep.CreateBooleanUnion(parts, 0.001)
    if not res or len(res) != 1:
        raise RuntimeError("union gave %d pieces" % (len(res) if res else 0))
    return res[0]


def bool_diff(a, b):
    res = rg.Brep.CreateBooleanDifference([a], b, 0.001)
    if not res or len(res) != 1:
        raise RuntimeError("difference gave %d pieces" % (len(res) if res else 0))
    return res[0]


def build():
    outer = cylinder(PX, PY, ZA, ZB - ZA, TUBE_R)
    inner = cylinder(PX, PY, ZA - 1.0, ZB - ZA + 2.0, TUBE_R - TUBE_WALL)
    parts = [bool_diff(outer, [inner])]
    holes = []
    for b in BOLTS:
        bid = b["id"]
        x, y = b["position"][0], b["position"][1]
        g = GRIP[bid]
        if bid.startswith("A"):
            parts.append(cylinder(x, y, ZA, g, A_BOSS_R))
            parts.append(arm(PX, PY, x, y, A_ARM_W, ZA, g))
            holes.append(cylinder(x, y, ZA - 1.0, g + 2.0, 0.5 * A_HOLE_D))
        else:
            parts.append(cylinder(x, y, ZB - g, g, B_BOSS_R))
            parts.append(gusset(PX, PY, x, y, B_ARM_W, ZB, ZB - g))
            holes.append(cylinder(x, y, ZB - g - 1.0, g + 2.0, 0.5 * B_HOLE_D))
    body = bool_union(parts)
    body = bool_diff(body, holes)
    body.MergeCoplanarFaces(0.001)
    return body


def plane_groups(brep, lines):
    groups = {}
    for f in brep.Faces:
        ok, pl = f.TryGetPlane(0.01)
        if not ok:
            continue
        n = pl.Normal
        d = pl.Origin.X * n.X + pl.Origin.Y * n.Y + pl.Origin.Z * n.Z
        key = (round(n.X, 2), round(n.Y, 2), round(n.Z, 2), round(d, 2))
        groups.setdefault(key, []).append(round(rg.AreaMassProperties.Compute(f).Area, 1))
    for k, v in sorted(groups.items()):
        if len(v) > 1:
            lines.append("PLANE n=(%.2f,%.2f,%.2f) d=%.2f faces=%d areas=%s" % (k[0], k[1], k[2], k[3], len(v), v))


def mesh_of(brep):
    m = rg.Mesh()
    for part in rg.Mesh.CreateFromBrep(brep, rg.MeshingParameters.Default):
        m.Append(part)
    return m


def report(brep, lines):
    lines.append("valid=%s solid=%s" % (brep.IsValid, brep.IsSolid))
    plane_groups(brep, lines)
    lines.append("volume_mm3=%.1f faces=%d" % (brep.GetVolume(), brep.Faces.Count))
    bb = brep.GetBoundingBox(True)
    lines.append("bbox min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)" % (bb.Min.X, bb.Min.Y, bb.Min.Z, bb.Max.X, bb.Max.Y, bb.Max.Z))
    mesh = mesh_of(brep)
    for b in BOLTS:
        bid = b["id"]
        x, y = b["position"][0], b["position"][1]
        g = GRIP[bid]
        if bid.startswith("A"):
            top = ZA + g + 4.0            # head top
            bend = top - 2.0 + 20.0       # socket bottom + short leg
            head_r = 3.5
            tool_z = [top + 0.5 + i for i in range(30)]
        else:
            bot = ZB - g - 5.0            # head bottom
            bend = bot + 2.0 - 20.0
            head_r = 4.25
            tool_z = [bot - 0.5 - i for i in range(30)]
        zs = [bend - 2.0, bend, bend + 2.0]
        free = []
        for k in range(120):
            th = math.radians(3.0 * k)
            blocked = False
            r = 6.0
            while r < 64.0 and not blocked:
                for z in zs:
                    p = rg.Point3d(x + r * math.cos(th), y + r * math.sin(th), z)
                    if mesh.IsPointInside(p, 0.001, False):
                        blocked = True
                        break
                r += 1.0
            free.append(not blocked)
        best = 0
        run = 0
        for ok in free + free:
            run = run + 1 if ok else 0
            best = max(best, run)
        best = min(best, 120)
        lines.append("KEY %s free window %.0f deg (need 60)" % (bid, best * 3.0))
        hit = False
        for z in tool_z:
            for rr in (0.0, 0.5 * head_r, head_r):
                for k in range(8):
                    th = math.radians(45.0 * k)
                    p = rg.Point3d(x + rr * math.cos(th), y + rr * math.sin(th), z)
                    if mesh.IsPointInside(p, 0.001, False):
                        hit = True
        lines.append("TOOL %s tool cylinder clear=%s" % (bid, not hit))


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    lines = []
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        brep = build()
        report(brep, lines)
        doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    lines.insert(0, "ok: wrote output.step")
    open(LOG, "w").write("\n".join(lines) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
