#! python3
"""Connecting part between fixed mount A (below, square, 4x M4) and fixed mount B (above, round, 3x M5).

Design: a hollow central tube sits between the two mounting faces, with a flat
spider of bosses at each end. The A spider (grip 3.5) lies on face A, the B
spider (grip 6.5) hangs under face B. Every bolt head sits on a boss; the
spider arms run out from the tube, so the space above the A heads and below
the B heads, and the 60-degree key swings out from each bolt, stay clear.

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

# Mounting faces (world Z). Part A: face normal +z, part sits above it.
# Part B: face normal -z, part sits below it.
ZA = -94.0
ZB = -25.5

# Tube between the faces: centre between the two mount centres.
CA = (-126.0, 127.5)
CB = (-124.18164228311065, 135.11206773573574)
PX = 0.5 * (CA[0] + CB[0])
PY = 0.5 * (CA[1] + CB[1])

TUBE_R = 12.0
TUBE_WALL = 1.2

A_GRIP = 3.5
A_BOSS_R = 4.0
A_HOLE_D = 4.5
A_ARM_W = 7.0

B_GRIP = 6.5
B_BOSS_R = 5.0
B_HOLE_D = 5.5
B_ARM_W = 8.0

ARM_START = 11.0  # arms start inside the tube wall (wall is 10.8 .. 12.0)


def cylinder(cx, cy, z0, h, r):
    circ = rg.Circle(rg.Plane(rg.Point3d(cx, cy, z0), rg.Vector3d.ZAxis), r)
    return rg.Cylinder(circ, h).ToBrep(True, True)


def arm(px, py, tx, ty, width, z0, h):
    """Horizontal rectangular arm from the tube axis towards (tx, ty)."""
    dx, dy = tx - px, ty - py
    length = math.hypot(dx, dy)
    ax, ay = dx / length, dy / length
    pl = rg.Plane(rg.Point3d(px, py, z0),
                  rg.Vector3d(ax, ay, 0.0),
                  rg.Vector3d(-ay, ax, 0.0))
    box = rg.Box(pl, rg.Interval(ARM_START, length),
                 rg.Interval(-0.5 * width, 0.5 * width),
                 rg.Interval(0.0, h))
    return box.ToBrep()


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
    # Tube (hollow) from face A to face B.
    outer = cylinder(PX, PY, ZA, ZB - ZA, TUBE_R)
    inner = cylinder(PX, PY, ZA - 1.0, ZB - ZA + 2.0, TUBE_R - TUBE_WALL)
    tube = bool_diff(outer, [inner])

    parts = [tube]
    holes = []
    for b in BOLTS:
        x, y = b["position"][0], b["position"][1]
        if b["id"].startswith("A"):
            parts.append(cylinder(x, y, ZA, A_GRIP, A_BOSS_R))
            parts.append(arm(PX, PY, x, y, A_ARM_W, ZA, A_GRIP))
            holes.append(cylinder(x, y, ZA - 1.0, A_GRIP + 2.0, 0.5 * A_HOLE_D))
        else:
            parts.append(cylinder(x, y, ZB - B_GRIP, B_GRIP, B_BOSS_R))
            parts.append(arm(PX, PY, x, y, B_ARM_W, ZB - B_GRIP, B_GRIP))
            holes.append(cylinder(x, y, ZB - B_GRIP - 1.0, B_GRIP + 2.0, 0.5 * B_HOLE_D))

    body = bool_union(parts)
    body = bool_diff(body, holes)
    body.MergeCoplanarFaces(0.001)
    return body


def mesh_of(brep):
    m = rg.Mesh()
    for part in rg.Mesh.CreateFromBrep(brep, rg.MeshingParameters.Default):
        m.Append(part)
    return m


def report(brep, lines):
    lines.append("valid=%s solid=%s" % (brep.IsValid, brep.IsSolid))
    lines.append("volume_mm3=%.1f faces=%d" % (brep.GetVolume(), brep.Faces.Count))
    bb = brep.GetBoundingBox(True)
    lines.append("bbox min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)" % (bb.Min.X, bb.Min.Y, bb.Min.Z, bb.Max.X, bb.Max.Y, bb.Max.Z))
    mesh = mesh_of(brep)
    # Key swing: long leg of the L-key at the bend height, radius 64 from the axis.
    for b in BOLTS:
        x, y = b["position"][0], b["position"][1]
        if b["id"].startswith("A"):
            top = ZA + A_GRIP + 4.0           # head top
            bend = top - 2.0 + 20.0           # socket bottom + short leg
            sign = 1.0
            head_r = 3.5
            zs = [bend - 2.0, bend, bend + 2.0]
        else:
            bot = ZB - B_GRIP - 5.0           # head bottom
            bend = bot + 2.0 - 20.0
            sign = -1.0
            head_r = 4.25
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
        lines.append("KEY %s free window %.0f deg (need 60)" % (b["id"], best * 3.0))
        # Tool cylinder above (A) / below (B) head, 30 mm long, head radius.
        hit = False
        for i in range(30):
            z = (top + 0.5 + i) if sign > 0 else (bot - 0.5 - i)
            for rr in (0.0, 0.5 * head_r, head_r):
                for k in range(8):
                    th = math.radians(45.0 * k)
                    p = rg.Point3d(x + rr * math.cos(th), y + rr * math.sin(th), z)
                    if mesh.IsPointInside(p, 0.001, False):
                        hit = True
        lines.append("TOOL %s tool cylinder clear=%s" % (b["id"], not hit))


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
