#! python3
"""Bracket between round flange A and rectangular block B (task flange-L2-s1).

Concept (printed standing, +Z up, no supports; everything is a vertical wall):
  * A side: hexagonal plate (corners on the six M5 bolts) with a 45-degree
    chamfered seat pad, carried down to the print bed by a leg.
  * B side: rectangular plate, 1 mm inside the block's face.
  * One web, parallel to the B bolt axes, joins both plates over the full height.
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
MSG = []

# ---- design numbers (mm) -------------------------------------------------
INSET = 1.05          # seat stays this far inside each mounting face
PAD = 2.0             # height of the chamfered seat pad on A
T_A = 8.0             # total thickness at A (grip of the M5 bolts)
T_B = 8.0             # thickness of the B plate (grip of the M6 bolts)
T_WEB = 8.0
WEB_SHIFT = 3.5       # web off the B centre line, gives the A-side keys room
D_A, D_B = 5.5, 6.6   # clearance holes
A_DIA, B_W, B_H = 74.5, 46.0, 96.0

EZ = rg.Vector3d(0, 0, 1)
a0 = rg.Point3d(*FACES["A"]["origin"]); nA = rg.Vector3d(*FACES["A"]["normal"]); nA.Unitize()
b0 = rg.Point3d(*FACES["B"]["origin"]); nB = rg.Vector3d(*FACES["B"]["normal"]); nB.Unitize()
tA = rg.Vector3d(nA.Y, -nA.X, 0)   # horizontal, in face A
tB = rg.Vector3d(-nB.Y, nB.X, 0)   # horizontal, in face B
if tB * tA < 0:
    tB = -tB

Z_BED = b0.Z - B_H / 2 + INSET     # bottom of the part = print bed
Z_TOPB = b0.Z + B_H / 2 - INSET


def PA(u, v, z):
    return a0 + tA * u + nA * v + EZ * (z - a0.Z)


def PB(s, w, z):
    return b0 + tB * s + nB * w + EZ * (z - b0.Z)


def planar(pts):
    pl = rg.Polyline(list(pts) + [pts[0]]).ToNurbsCurve()
    return rg.Brep.CreatePlanarBreps(pl, TOL)[0]


def prism(plan_pts, z0, z1):
    """Closed prism from plan points (Point3d, z ignored)."""
    bot = [rg.Point3d(p.X, p.Y, z0) for p in plan_pts]
    top = [rg.Point3d(p.X, p.Y, z1) for p in plan_pts]
    n = len(bot)
    faces = [planar(bot), planar(top)]
    for i in range(n):
        j = (i + 1) % n
        faces.append(planar([bot[i], bot[j], top[j], top[i]]))
    return join(faces)


def join(faces):
    j = rg.Brep.JoinBreps(faces, TOL)
    if len(j) != 1 or not j[0].IsSolid:
        raise RuntimeError("join did not give one solid")
    b = j[0]
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def plate_A():
    r1 = A_DIA / 2 - INSET              # seat hexagon, corners on the bolt directions
    h1 = r1 * math.sqrt(3) / 2
    h2 = h1 + PAD                       # 45 degree chamfer round the pad
    r2 = h2 * 2 / math.sqrt(3)
    zc = a0.Z

    def hexa(r, h, v):
        return [PA(r, v, zc), PA(r / 2, v, zc + h), PA(-r / 2, v, zc + h),
                PA(-r, v, zc), PA(-r / 2, v, zc - h), PA(r / 2, v, zc - h)]
    f0 = hexa(r1, h1, 0.0)
    f2 = hexa(r2, h2, PAD)

    def outline(v):  # hexagon with the leg down to the bed
        h = hexa(r2, h2, v)
        return h[:5] + [PA(-r2 / 2, v, Z_BED), PA(r2 / 2, v, Z_BED)] + h[5:]
    o2, o8 = outline(PAD), outline(T_A)
    faces = [planar(f0), planar(o8)]
    for i in range(6):
        j = (i + 1) % 6
        faces.append(planar([f0[i], f0[j], f2[j], f2[i]]))
    faces.append(planar([o2[4], o2[5], o2[6], o2[7]]))      # front of the leg
    for i in range(8):
        j = (i + 1) % 8
        faces.append(planar([o2[i], o2[j], o8[j], o8[i]]))
    return join(faces)


def plate_B():
    hw = B_W / 2 - INSET
    return prism([PB(-hw, 0, 0), PB(hw, 0, 0), PB(hw, T_B, 0), PB(-hw, T_B, 0)], Z_BED, Z_TOPB)


def web():
    v_end = (PAD + T_A) / 2             # ends inside the A plate

    def w_at(s):                        # distance from B where v == v_end
        base = (rg.Vector3d(b0 - a0) + tB * s) * nA
        return (v_end - base) / (nB * nA)
    s0, s1 = WEB_SHIFT - T_WEB / 2, WEB_SHIFT + T_WEB / 2
    pts = [PB(s0, T_B / 2, 0), PB(s1, T_B / 2, 0), PB(s1, w_at(s1), 0), PB(s0, w_at(s0), 0)]
    return prism(pts, Z_BED, Z_TOPB)


def build():
    parts = [plate_A(), plate_B(), web()]
    u = rg.Brep.CreateBooleanUnion(parts, TOL)
    if u is None or len(u) != 1:
        raise RuntimeError("union failed")
    body = u[0]
    body.MergeCoplanarFaces(TOL)
    cutters = []
    for b in BOLTS:
        d = D_A if b["size"] == "M5" else D_B
        ax = rg.Vector3d(*b["axis"]); ax.Unitize()
        p = rg.Point3d(*b["position"]) - ax * 5
        cyl = rg.Cylinder(rg.Circle(rg.Plane(p, ax), d / 2), 30)
        cutters.append(cyl.ToBrep(True, True))
    r = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    if r is None or len(r) != 1:
        raise RuntimeError("holes failed")
    body = r[0]
    body.MergeCoplanarFaces(TOL)
    body.Compact()
    ok, log = body.IsValidWithLog()
    vol = rg.VolumeMassProperties.Compute(body).Volume
    bb = body.GetBoundingBox(True)
    MSG.append("solid=%s valid=%s faces=%d edges=%d volume=%.0f mm3" % (
        body.IsSolid, ok, body.Faces.Count, body.Edges.Count, vol))
    MSG.append("bbox %s .. %s" % (bb.Min, bb.Max))
    MSG.append("min edge %.3f" % min(e.GetLength() for e in body.Edges))
    if not (body.IsSolid and ok):
        raise RuntimeError("bad solid: " + log)
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
    open(LOG, "w").write("error:\n" + err + "\n".join(MSG) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
