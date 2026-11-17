#! python3
"""Connecting part between mount A (square, 4x M4 above) and mount B (round, 3x M5 below).

Design (world mm):
- Plate under A: 68 x 68 square plate, 8 mm thick, rotated 35 deg like the A bolt pattern.
  It sits on A's face (z=-94); the four M4 heads sit on its top face.
- Disc over B: 80 mm dia disc, 7 mm thick, centred on B. It sits on B's face (z=-25.5);
  the three M5 heads sit under its underside.
- Column: a 22 mm radius tube with a 59 deg flare into the disc. The flare stays inside
  radius 29.5 so it clears the M5 heads (r=35 from B centre, head r=4.25 + 0.5 clearance).
  Its outer surface is at most 59 deg from horizontal, so it prints without support.
- Clearance holes: 4.5 mm (M4) and 5.5 mm (M5), through their own plate or disc.
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
FACES = POSE.get("mounting_faces", [])
BOLTS = POSE.get("bolts", [])

FACE_A = next(f for f in FACES if f["part"] == "A")
FACE_B = next(f for f in FACES if f["part"] == "B")
ZA = FACE_A["origin"][2]          # -94.0, top face of mount A
ZB = FACE_B["origin"][2]          # -25.5, bottom face of mount B
CA = (FACE_A["origin"][0], FACE_A["origin"][1])
CB = (FACE_B["origin"][0], FACE_B["origin"][1])
THETA_A = math.radians(35.0)      # square face rotation (from the M4 hole pattern)

PLATE_T = 8.0                     # grip under M4 heads (range 3-25)
DISC_T = 7.0                      # grip under M5 heads (range 3.75-25)
PLATE_HALF = 34.0                 # inside the 73 mm square less 1 mm setback (35.5)
DISC_R = 40.0                     # inside the 84.5 mm round less 1 mm setback (41.25)
COL_R = 22.0
COL_FLARE_R = 29.5
FLARE_Z = -45.0                   # flare start
D4 = 4.5                          # M4 clearance hole
D5 = 5.5                          # M5 clearance hole


def pt(x, y, z):
    return rg.Point3d(x, y, z)


def circle_curve(x, y, z, r):
    return rg.Circle(rg.Plane(pt(x, y, z), rg.Vector3d.ZAxis), r).ToNurbsCurve()


def cyl(x, y, z0, h, r):
    """Solid cylinder from z0 up by h."""
    return rg.Extrusion.Create(circle_curve(x, y, z0, r), h, True).ToBrep()


def build():
    tol = 0.001
    # plate: rotated square under A
    corners = []
    for lx, ly in [(-PLATE_HALF, -PLATE_HALF), (PLATE_HALF, -PLATE_HALF),
                   (PLATE_HALF, PLATE_HALF), (-PLATE_HALF, PLATE_HALF)]:
        c, s = math.cos(THETA_A), math.sin(THETA_A)
        corners.append(pt(CA[0] + lx * c - ly * s, CA[1] + lx * s + ly * c, ZA))
    corners.append(corners[0])
    plate_curve = rg.PolylineCurve(corners)
    plate = rg.Extrusion.Create(plate_curve, PLATE_T, True).ToBrep()

    # disc: round seat under B
    zd = ZB - DISC_T
    disc = rg.Extrusion.Create(circle_curve(CB[0], CB[1], zd, DISC_R), DISC_T, True).ToBrep()

    # column: straight tube, 59 deg flare (frustum) into the disc, short collar inside the disc
    def solid_of(curves):
        lofts = rg.Brep.CreateFromLoft(curves, rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Straight, False)
        if not lofts:
            raise RuntimeError("loft failed")
        capped = lofts[0].CapPlanarHoles(tol)
        if capped is None:
            raise RuntimeError("cap failed")
        return capped

    z_bot = ZA + PLATE_T - 4.0     # start inside plate
    col_lo = cyl(CB[0], CB[1], z_bot, FLARE_Z + 1.0 - z_bot, COL_R)
    # frustum from the 22 mm tube up to the disc, slope 0.6 mm radial per mm (59 deg from horizontal)
    z_top = zd + 1.0
    r_top = COL_R + 0.6 * (z_top - FLARE_Z)
    flare = solid_of([circle_curve(CB[0], CB[1], FLARE_Z, COL_R),
                      circle_curve(CB[0], CB[1], z_top, r_top)])

    def vol(b):
        return rg.VolumeMassProperties.Compute(b).Volume

    parts = [plate, col_lo, flare, disc]
    for p_ in parts:
        if p_.SolidOrientation == rg.BrepSolidOrientation.Inward:
            p_.Flip()
    dbg0 = ["vol %s %.1f orient %s" % (i, vol(p_), p_.SolidOrientation) for i, p_ in enumerate(parts)]
    open(os.path.join(HERE, "scratch", "dbg0.txt"), "w").write("\n".join(dbg0) + "\n")
    dbg = ["part %d v=%s s=%s f=%d" % (i, p_.IsValid, p_.IsSolid, p_.Faces.Count) for i, p_ in enumerate(parts)]
    body = parts[0]
    for p_ in parts[1:]:
        u = rg.Brep.CreateBooleanUnion([body, p_], tol)
        dbg.append("union -> %s" % ("none" if not u else str([(x.IsSolid, x.Faces.Count) for x in u])))
        if not u or len(u) != 1:
            open(os.path.join(HERE, "scratch", "dbg.txt"), "w").write("\n".join(dbg) + "\n")
            raise RuntimeError("union did not give one solid")
        body = u[0]
    open(os.path.join(HERE, "scratch", "dbg.txt"), "w").write("\n".join(dbg) + "\n")

    # clearance holes at the bolt positions
    cutters = []
    for b in BOLTS:
        x, y = b["position"][0], b["position"][1]
        if b["size"] == "M4":
            cutters.append(cyl(x, y, ZA - 2.0, PLATE_T + 4.0, D4 / 2.0))
        else:
            cutters.append(cyl(x, y, zd - 2.0, DISC_T + 4.0, D5 / 2.0))
    out = body
    dl = ["body v=%s s=%s bbox=%s" % (body.IsValid, body.IsSolid, str(body.GetBoundingBox(True)))]
    for i, c in enumerate(cutters):
        res = rg.Brep.CreateBooleanDifference([out], [c], tol)
        dl.append("cut %d: %s bbox=%s" % (i, "none" if not res else [(x.IsSolid, x.Faces.Count) for x in res], str(c.GetBoundingBox(True))))
        if not res or len(res) != 1:
            open(os.path.join(HERE, "scratch", "dbg.txt"), "w").write("\n".join(dl) + "\n")
            raise RuntimeError("hole cut did not give one solid")
        out = res[0]
    out.MergeCoplanarFaces(tol)
    if not out.IsSolid or not out.IsValid:
        raise RuntimeError("result is not a valid closed solid")
    return [out]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        breps = build()
        for brep in breps:
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
        vol = rg.VolumeMassProperties.Compute(breps[0])
        log = "ok: wrote output.step volume=%.1f mm3 faces=%d\n" % (vol.Volume if hasattr(vol, "Volume") else 0.0, breps[0].Faces.Count)
    finally:
        doc.Dispose()
    open(LOG, "w").write(log)


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
