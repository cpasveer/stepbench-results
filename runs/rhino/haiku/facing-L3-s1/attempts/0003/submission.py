#! python3
"""Connecting part between fixed mount A (below, square, 4x M4) and fixed mount B (above, round, 3x M5).

Design (printed as it comes off the printer, z up)
- A hollow tube (outer 40 mm, wall 1.2 mm) stands between the two mounting faces
  and carries the load. Its ends rest on face A and face B.
- Face A end: four flat arms run from the tube to four bosses on the A holes.
  The A head seats are flat and face up, so they print without support.
- Face B end: each B head seat is a flat bridge (9.6 mm span) over the head,
  carried at both ends by sloped supports, one of which runs down to the tube.
  Every underside is at 47 deg or steeper, so no support is needed.
- Keys and tools: the 60-degree key swing and the 30 mm straight tool are kept
  free from every bolt head.

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
ARM_START = TUBE_R - 0.6          # arms and gussets start inside the tube wall

# Grip per bolt (thickness under the head, mm). Distinct values, so that each
# bearing face is a plane of its own.
GRIP = {"A1": 3.5, "A2": 4.0, "A3": 3.2, "A4": 4.5,
        "B1": 6.5, "B2": 7.0, "B3": 6.0}

A_BOSS_R = 4.0
A_HOLE_D = 4.5
A_ARM_W = 7.0

B_HOLE_R = 2.75                   # M5 clearance 5.5
B_SPAN_HALF = 4.8                 # bridge half span: head radius 4.25 + 0.5 clearance + 0.05
B_RIB_OUT = 8.8                   # outer edge of the support on the far side
B_RIB_DROP = 6.5                  # the far-side support slopes down by this much
B_W = 9.0                         # width of the B seat structure (lateral)
SLOPE = math.tan(math.radians(47.0))  # underside slope of the tube-side gusset


def cylinder(cx, cy, z0, h, r, seam_dir=None):
    """Vertical cylinder. seam_dir (unit xy) puts the cylinder seam on that side."""
    if seam_dir is None:
        plane = rg.Plane(rg.Point3d(cx, cy, z0), rg.Vector3d.ZAxis)
    else:
        ux, uy = seam_dir
        plane = rg.Plane(rg.Point3d(cx, cy, z0),
                         rg.Vector3d(ux, uy, 0.0),
                         rg.Vector3d(-uy, ux, 0.0))
    circ = rg.Circle(plane, r)
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


def prism(ox, oy, u, pts_sz, width):
    """Extrude the vertical profile pts_sz = [(s, z)] laterally by width.
    s runs along the horizontal unit vector u from (ox, oy); z is world Z."""
    ux, uy = u
    pl = rg.Plane(rg.Point3d(ox, oy, 0.0), rg.Vector3d(ux, uy, 0.0), rg.Vector3d.ZAxis)
    pts = [rg.Point3d(ox, oy, 0.0) + pl.XAxis * s + pl.YAxis * z for s, z in pts_sz]
    pts.append(pts[0])
    crv = rg.Polyline(pts).ToNurbsCurve()
    lat = rg.Vector3d(-uy, ux, 0.0)
    crv.Transform(rg.Transform.Translation(lat * (-0.5 * width)))
    ext = rg.Extrusion.Create(crv, width, True).ToBrep()
    c = rg.VolumeMassProperties.Compute(ext).Centroid
    shift = -(lat.X * (c.X - ox) + lat.Y * (c.Y - oy))
    ext.Transform(rg.Transform.Translation(lat * shift))
    return ext


def b_seat(bx, by, grip):
    """Structure under one B bolt. Profile in the plane through the bolt axis and the tube."""
    lb = math.hypot(PX - bx, PY - by)
    u = ((PX - bx) / lb, (PY - by) / lb)        # s = 0 at the bolt, s = lb at the tube axis
    zb = ZB - grip                              # underside of the bridge (head top)
    s_t = lb - (TUBE_R - 0.6)                   # gusset ends inside the tube wall
    lip = 1.0                                   # vertical lip under each bridge end
    z_t = zb - lip - (s_t - B_SPAN_HALF) * SLOPE  # gusset underside meets the tube
    prof = [(-B_RIB_OUT, ZB), (s_t, ZB), (s_t, z_t), (B_SPAN_HALF, zb - lip),
            (B_SPAN_HALF, zb), (-B_SPAN_HALF, zb), (-B_SPAN_HALF, zb - lip),
            (-B_RIB_OUT, zb - B_RIB_DROP)]
    return prism(bx, by, u, prof, B_W)


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
        tx, ty = PX - x, PY - y
        tl = math.hypot(tx, ty)
        toward = (tx / tl, ty / tl)
        if bid.startswith("A"):
            parts.append(cylinder(x, y, ZA, g, A_BOSS_R, seam_dir=toward))
            parts.append(arm(PX, PY, x, y, A_ARM_W, ZA, g))
            holes.append(cylinder(x, y, ZA - 1.0, g + 2.0, 0.5 * A_HOLE_D))
        else:
            parts.append(b_seat(x, y, g))
            holes.append(cylinder(x, y, ZB - g - 1.0, g + 2.0, B_HOLE_R))
    body = bool_union(parts)
    body = bool_diff(body, holes)
    body.MergeCoplanarFaces(0.001)
    body.MergeCoplanarFaces(0.01)
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


OVERHANG_DIRS = [("+Z", (0, 0, 1)), ("-Z", (0, 0, -1)), ("+X", (1, 0, 0)),
                 ("-X", (-1, 0, 0)), ("+Y", (0, 1, 0)), ("-Y", (0, -1, 0))]


def overhang_screen(brep, lines):
    """Area of downward faces steeper than 45 deg that are not on the bed, per build direction.
    Flat bridges (both ends supported) are included here and must be judged separately."""
    mesh = mesh_of(brep)
    mesh.Normals.ComputeNormals()
    fn = mesh.FaceNormals
    for label, u in OVERHANG_DIRS:
        uv = rg.Vector3d(u[0], u[1], u[2])
        heights = [uv.X * v.X + uv.Y * v.Y + uv.Z * v.Z for v in mesh.Vertices]
        hmin = min(heights)
        area_fail = 0.0
        for i in range(mesh.Faces.Count):
            f = mesh.Faces[i]
            tris = [(f.A, f.B, f.C)] if not f.IsQuad else [(f.A, f.B, f.C), (f.A, f.C, f.D)]
            for (a, b, c) in tris:
                pa, pb, pc = mesh.Vertices[a], mesh.Vertices[b], mesh.Vertices[c]
                cr = rg.Vector3d.CrossProduct(pb - pa, pc - pa)
                area = 0.5 * cr.Length
                if area <= 0:
                    continue
                n = rg.Vector3d(fn[i].X, fn[i].Y, fn[i].Z)
                n.Unitize()
                if n * uv > -0.707:
                    continue
                hs = [heights[a] - hmin, heights[b] - hmin, heights[c] - hmin]
                if max(hs) < 0.3:
                    continue
                area_fail += area
                if label == "+Z" and area > 1.0:
                    cen = (pa + pb + pc) / 3.0
                    lines.append("  +Z steep face at (%.1f,%.1f,%.1f) area %.1f" % (cen.X, cen.Y, cen.Z, area))
        lines.append("OVERHANG %s steep-down area not on bed = %.0f mm2" % (label, area_fail))


def report(brep, lines):
    lines.append("valid=%s solid=%s" % (brep.IsValid, brep.IsSolid))
    plane_groups(brep, lines)
    overhang_screen(brep, lines)
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
