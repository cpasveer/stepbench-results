#! python3
"""Connecting bracket between round pipe A and flat square mount B.

Design (mm, world coordinates):
  * pad A: saddle on the 68 mm pipe. Inner face is the true pipe cylinder (r 34),
    top is a flat seat at r 40 under each of the four M5 heads (two bolt rows 70 deg apart),
    extruded 48 mm along the pipe so all four holes sit in it.
  * pad B: 35 x 35 x 8 plate lying flat on the 38 mm square mount, inside its outline.
  * rib: 8 x 5 section bar rising from pad B to saddle A, kept between the M4 heads
    and clear of the head/key zones.
  * 4 x M5 clearance 5.5 and 4 x M4 clearance 4.5 holes on the bolt axes.
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
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]

MSG = []

TA = 6.0      # pad A thickness under the bolt head (grip)
TB = 8.0      # pad B thickness (grip)
S_HALF = 24.0  # half length of pad A along the pipe axis
RI = 34.0     # pipe outside radius
TOL = 0.01


def V(x, y, z):
    return rg.Vector3d(x, y, z)


def unit(v):
    v = rg.Vector3d(v)
    v.Unitize()
    return v


def cross(a, b):
    return rg.Vector3d.CrossProduct(a, b)


def dot(a, b):
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z


def polar(rad, deg):
    th = math.radians(deg)
    return rad * math.cos(th), rad * math.sin(th)


def build():
    # ---- pipe frame: axis dA, cross-section basis (r1, t1) ----
    P0 = rg.Point3d(21.0, 136.5, -7.0)
    dA = unit(V(0.2487956429525063, 0.5900653629615055, 0.7680648380709507))
    r1 = unit(V(*BOLTS["A1"]["axis"]))
    r3 = unit(V(*BOLTS["A3"]["axis"]))
    t1 = unit(cross(dA, r1))
    x3 = dot(r3, r1)
    y3 = dot(r3, t1)
    MSG.append("pipe: r1.dA=%.5f r3.dA=%.5f angle3=%.2f deg" % (
        dot(r1, dA), dot(r3, dA), math.degrees(math.atan2(y3, x3))))

    def pp(a, b, s):
        return P0 + r1 * a + t1 * b + dA * s

    # ---- pad A: saddle on the pipe ----
    S0 = -S_HALF
    Ro = RI + TA
    w = 7.0
    Ai = pp(*polar(RI, -85.0), S0)
    Am = pp(*polar(RI, -36.5), S0)
    Bi = pp(*polar(RI, 12.0), S0)
    F1p = pp(Ro, w, S0)
    F1m = pp(Ro, -w, S0)
    M = pp(*polar(Ro, -35.0), S0)
    F3p = pp(Ro * x3 - w * y3, Ro * y3 + w * x3, S0)
    F3m = pp(Ro * x3 + w * y3, Ro * y3 - w * x3, S0)
    segs = [rg.ArcCurve(rg.Arc(Ai, Am, Bi)),
            rg.LineCurve(Bi, F1p), rg.LineCurve(F1p, F1m), rg.LineCurve(F1m, M),
            rg.LineCurve(M, F3p), rg.LineCurve(F3p, F3m), rg.LineCurve(F3m, Ai)]
    joined = rg.Curve.JoinCurves(segs, 0.01)
    prof = joined[0]
    if not prof.IsClosed:
        raise RuntimeError("pad A profile not closed")
    ext = rg.Extrusion.Create(prof, 2 * S_HALF, True).ToBrep()
    c = ext.GetBoundingBox(True).Center
    sc = dot(c - P0, dA)
    if abs(sc) > 1.0:
        ext.Transform(rg.Transform.Translation(dA * (-sc)))
    padA = ext

    # ---- pad B: plate on the mounting face ----
    O_B = rg.Point3d(*FACE["origin"])
    nB = unit(V(*FACE["normal"]))
    uB = unit(V(0.9214414224885092, -0.3885173161164798, 0.0))
    zneg = V(0.0, 0.0, -1.0)
    planeB = rg.Plane(O_B, uB, zneg)  # normal = nB
    padB = rg.Box(planeB, rg.Interval(-17.5, 17.5), rg.Interval(-17.5, 17.5),
                  rg.Interval(0.0, TB)).ToBrep()

    # ---- rib from pad B edge to pad A ----
    E_A = pp(*polar(37.0, -35.0), 0.0)
    E_B = O_B + nB * 4.0 + V(0.0, 0.0, -12.0)
    vec = E_A - E_B
    L = vec.Length
    Vu = unit(vec)
    rE = unit(E_A - P0)
    wdir = unit(cross(Vu, rE))
    hdir = unit(cross(Vu, wdir))
    hw, hh = 4.0, 2.5
    corners = [E_B + wdir * (-hw) + hdir * (-hh), E_B + wdir * hw + hdir * (-hh),
               E_B + wdir * hw + hdir * hh, E_B + wdir * (-hw) + hdir * hh]
    corners.append(corners[0])
    rect = rg.PolylineCurve(corners)
    rib = rg.Extrusion.Create(rect, L, True).ToBrep()
    c = rib.GetBoundingBox(True).Center
    if dot(c - E_B, Vu) < 0:
        rib.Transform(rg.Transform.Translation(Vu * L))

    parts = [padA, padB, rib]
    res = rg.Brep.CreateBooleanUnion(parts, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("union gave %s breps" % (0 if not res else len(res)))
    body = res[0]

    # ---- bolt holes on every bolt axis ----
    cutters = []
    for bid in ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]:
        b = BOLTS[bid]
        pos = rg.Point3d(*b["position"])
        ax = unit(V(*b["axis"]))
        d = 5.5 if b["size"] == "M5" else 4.5
        cyl = rg.Cylinder(rg.Circle(rg.Plane(pos - ax * 5.0, ax), d / 2.0), 40.0)
        cutters.append(cyl.ToBrep(True, True))
    diff = rg.Brep.CreateBooleanDifference([body], cutters, TOL)
    if not diff or len(diff) != 1:
        raise RuntimeError("hole cut gave %s breps" % (0 if not diff else len(diff)))
    body = diff[0]
    body.MergeCoplanarFaces(TOL)

    # ---- diagnostics ----
    vm = rg.VolumeMassProperties.Compute(body)
    bb = body.GetBoundingBox(True)
    MSG.append("valid=%s solid=%s faces=%d volume=%.1f" % (
        body.IsValid, body.IsSolid, body.Faces.Count, vm.Volume))
    MSG.append("bbox min=%s max=%s" % (bb.Min, bb.Max))
    pipeCyl = rg.Cylinder(rg.Circle(rg.Plane(P0 + dA * (-140.0), dA), RI), 280.0).ToBrep(True, True)
    inter = rg.Brep.CreateBooleanIntersection([body], [pipeCyl], TOL)
    ov = sum(rg.VolumeMassProperties.Compute(x).Volume for x in (inter or []))
    MSG.append("overlap with pipe volume=%.4f" % ov)
    return body


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    body = build()
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.Objects.AddBrep(body)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    MSG.append("ok: wrote output.step")
    open(LOG, "w").write("\n".join(MSG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("\n".join(MSG) + "\nerror:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
