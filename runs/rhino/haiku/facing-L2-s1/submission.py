#! python3
"""Connecting part between mount A (square, 4x M4 from below) and mount B (round, 3x M5 from above).

Design (world mm), printed as it stands (plate on the bed, Z up):
- Base plate under A: 68 x 68 mm square, 8 mm thick, turned 35 deg like the M4 pattern.
  It sits on A's face at z=-94; the four M4 heads sit on its top face.
- Central tube: 44 mm dia, from the plate up to B's face at z=-25.5, on B's centre.
- Three ribs (tripod) from the tube out to the three M5 bolt positions. Each rib is 10 mm wide,
  7 mm thick at its bolt pad, and its underside slopes at about 50 deg from the pad inward, so
  nothing hangs unsupported. The flat bearing pad under each M5 head is only 8.75 mm long.
  Nothing sits inside the head envelopes (head radius 4.25 + 0.5 mm clearance).
- Clearance holes: 4.5 mm (M4) through the plate, 5.5 mm (M5) through the rib pads.
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
PLATE_HALF = 34.0                 # inside the 73 mm square less 1 mm setback (35.5)
TUBE_R = 22.0
RIB_W = 10.0
RIB_T = 7.0                       # grip under M5 heads (range 3.75-25)
RIB_END = 40.5                    # radial reach from B centre (inside 41.25 setback at 10 mm width)
PAD_START = 30.25                 # flat bearing pad inner end (M5 hole at 35, head clearance 4.75)
PAD_END = 39.75                   # flat bearing pad outer end, where the lip starts
LIP_Z = -36.5                     # bottom of the lip that carries the outer end of the pad
RIB_SLOPE = 1.2                   # dz/ds of the rib underside (about 50 deg from horizontal)
D4 = 4.5                          # M4 clearance hole
D5 = 5.5                          # M5 clearance hole
TOL = 0.001


def pt(x, y, z):
    return rg.Point3d(x, y, z)


def circle_curve(x, y, z, r):
    return rg.Circle(rg.Plane(pt(x, y, z), rg.Vector3d.ZAxis), r).ToNurbsCurve()


def cyl(x, y, z0, h, r):
    return rg.Extrusion.Create(circle_curve(x, y, z0, r), h, True).ToBrep()


def plate_brep():
    corners = []
    c, s = math.cos(THETA_A), math.sin(THETA_A)
    for lx, ly in [(-PLATE_HALF, -PLATE_HALF), (PLATE_HALF, -PLATE_HALF),
                   (PLATE_HALF, PLATE_HALF), (-PLATE_HALF, PLATE_HALF)]:
        corners.append(pt(CA[0] + lx * c - ly * s, CA[1] + lx * s + ly * c, ZA))
    corners.append(corners[0])
    return rg.Extrusion.Create(rg.PolylineCurve(corners), PLATE_T, True).ToBrep()


def rib_brep(phi):
    """Rib in the vertical plane through B's centre at azimuth phi, 10 mm wide."""
    u = (math.cos(phi), math.sin(phi), 0.0)     # radial direction
    w = (math.sin(phi), -math.cos(phi), 0.0)    # lateral direction
    zp = ZB - RIB_T
    prof = [(0.0, ZB), (RIB_END, ZB), (RIB_END, LIP_Z), (PAD_END, zp),
            (PAD_START, zp), (0.0, zp - RIB_SLOPE * PAD_START)]
    pts = []
    for s_, z_ in prof:
        pts.append(pt(CB[0] + s_ * u[0], CB[1] + s_ * u[1], z_))
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    # place the profile on the lateral centre plane, then centre the extrusion on it
    br = rg.Extrusion.Create(crv, RIB_W, True).ToBrep()
    centre = br.GetBoundingBox(True).Center
    lat = (centre.X - CB[0]) * w[0] + (centre.Y - CB[1]) * w[1]
    br.Translate(rg.Vector3d(-lat * w[0], -lat * w[1], 0.0))
    return br


def build():
    base = plate_brep()
    tube = cyl(CB[0], CB[1], ZA + PLATE_T - 4.0, ZB - (ZA + PLATE_T - 4.0), TUBE_R)

    ribs = []
    for b in BOLTS:
        if b["size"] != "M5":
            continue
        phi = math.atan2(b["position"][1] - CB[1], b["position"][0] - CB[0])
        ribs.append(rib_brep(phi))

    parts = [base, tube] + ribs
    body = parts[0]
    for p_ in parts[1:]:
        u = rg.Brep.CreateBooleanUnion([body, p_], TOL)
        if not u or len(u) != 1:
            raise RuntimeError("union did not give one solid")
        body = u[0]

    cutters = []
    for b in BOLTS:
        x, y = b["position"][0], b["position"][1]
        if b["size"] == "M4":
            cutters.append(cyl(x, y, ZA - 2.0, PLATE_T + 4.0, D4 / 2.0))
        else:
            cutters.append(cyl(x, y, ZB - RIB_T - 2.0, RIB_T + 4.0, D5 / 2.0))
    out = body
    for c in cutters:
        res = rg.Brep.CreateBooleanDifference([out], [c], TOL)
        if not res or len(res) != 1:
            raise RuntimeError("hole cut did not give one solid")
        out = res[0]
    out.MergeCoplanarFaces(TOL)
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
        log = "ok: wrote output.step volume=%.1f mm3 faces=%d\n" % (vol.Volume, breps[0].Faces.Count)
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
