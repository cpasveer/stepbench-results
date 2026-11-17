#! python3
"""Your answer to this task, modelled in Rhino.

Write ONE solid to output.step (next to this file) that bolts to the fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely.

    sh run_rhino.sh                                  # runs this script inside Rhino, writes output.step
    jointbench verify . --by <your name> --tool rhino

The script builds the part in its own invisible Rhino document (millimetres,
world coordinates) and exports it as STEP. Its messages and any error go to
submission.log; on an error no output.step is left behind.
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
FACES = POSE.get("mounting_faces", [])  # [{"part", "origin", "normal"}]
BOLTS = POSE.get("bolts", [])  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]


def V(x, y, z=0.0):
    return rg.Vector3d(x, y, z)


def extrude_poly(pts, vec):
    c = rg.PolylineCurve(pts + [pts[0]])
    srf = rg.Surface.CreateExtrusion(c, vec)
    b = srf.ToBrep()
    return b.CapPlanarHoles(0.001)


def stadium(p0, p1, r, z0, z1):
    """Vertical prism whose plan is a stadium from p0 to p1 with radius r."""
    d = rg.Vector3d(p1[0] - p0[0], p1[1] - p0[1], 0)
    L = d.Length
    d.Unitize()
    n = rg.Vector3d(-d.Y, d.X, 0)
    a = rg.Point3d(p0[0], p0[1], z0)
    b = rg.Point3d(p1[0], p1[1], z0)
    pc = rg.PolyCurve()
    pc.Append(rg.Line(a + n * (-r), b + n * (-r)))
    pc.Append(rg.Arc(b + n * (-r), b + d * r, b + n * r))
    pc.Append(rg.Line(b + n * r, a + n * r))
    pc.Append(rg.Arc(a + n * r, a - d * r, a + n * (-r)))
    srf = rg.Surface.CreateExtrusion(pc, V(0, 0, z1 - z0))
    b = srf.ToBrep().CapPlanarHoles(0.001)
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(x, y, z0, z1, r):
    c = rg.Cylinder(rg.Circle(rg.Plane(rg.Point3d(x, y, z0), rg.Vector3d.ZAxis), r), z1 - z0)
    return c.ToBrep(True, True)


def square_plate(x, y, z0, z1, h, ang):
    ca, sa = math.cos(ang), math.sin(ang)
    pts = []
    for (u, v) in [(-h, -h), (h, -h), (h, h), (-h, h)]:
        pts.append(rg.Point3d(x + u * ca - v * sa, y + u * sa + v * ca, z0))
    b = extrude_poly(pts, V(0, 0, z1 - z0))
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def union(parts):
    res = rg.Brep.CreateBooleanUnion(parts, 0.001)
    return res


def build():
    tol = 0.001
    zA = FACES[0]["origin"][2]
    zB = FACES[1]["origin"][2]
    cA = FACES[0]["origin"]
    cB = FACES[1]["origin"]
    cx = cB[0]; cy = cB[1]
    RC = 12.0
    tA, tB = 5.0, 6.0
    channels = []
    pieces = [cyl(cx, cy, zA, zB, RC), square_plate(cx, cy, zA, zA + tA, 15.0, math.radians(35.0))]
    for b in BOLTS:
        p = b["position"]
        if b["id"].startswith("A"):
            pieces.append(stadium((cx, cy), p, 5.0, zA, zA + tA))
        else:
            r = 6.2
            # profile in arm plane: s along arm, gusset at 45 deg under arm
            d = rg.Vector3d(p[0] - cx, p[1] - cy, 0); d.Unitize()
            n = rg.Vector3d(-d.Y, d.X, 0)
            s1 = 27.0
            sE = 40.0
            zb = zB - tB
            prof = [(0, zb), (s1, zb), (s1, zB), (0, zB)]
            sw = 44.0
            prof = [(0, zb - sw), (sE, zb - (sw - sE)), (sE, zB), (0, zB)]
            o = rg.Point3d(cx, cy, 0)
            pts = [o + d * s + n * (-r) + rg.Vector3d(0, 0, z) for (s, z) in prof]
            pr = extrude_poly(pts, n * (2 * r))
            pieces.append(pr)
            sc = 24.0
            chn = extrude_poly([o + d * sc + n * (-4.8) + V(0, 0, zb),
                                o + d * (sE + 3) + n * (-4.8) + V(0, 0, zb),
                                o + d * (sE + 3) + n * (-4.8) + V(0, 0, zb - 24),
                                o + d * sc + n * (-4.8) + V(0, 0, zb - 24)], n * 9.6)
            if chn.SolidOrientation == rg.BrepSolidOrientation.Inward:
                chn.Flip()
            channels.append(chn)
    dbg = []
    for q in pieces:
        dbg.append((q.IsSolid, round(q.GetVolume(),1), q.SolidOrientation.ToString()))
    open(LOG+'.dbg','w').write(repr(dbg))
    u = union(pieces)
    open(LOG+'.dbg','a').write(' union n=%d' % (len(u) if u else -1))
    body = u[0]
    body.MergeCoplanarFaces(tol)
    cuts = list(channels)
    for b in BOLTS:
        p = b["position"]
        dia = 4.5 if b["size"] == "M4" else 5.5
        if b["id"].startswith("A"):
            cuts.append(cyl(p[0], p[1], zA - 1, zA + tA + 1, dia / 2))
        else:
            cuts.append(cyl(p[0], p[1], zB - tB - 1, zB + 1, dia / 2))
    res = rg.Brep.CreateBooleanDifference([body], cuts, tol)
    out = res[0]
    out.MergeCoplanarFaces(tol)
    lines = []
    def key(f):
        ok, pl = f.TryGetPlane(0.001)
        if ok:
            n = pl.Normal
            if f.OrientationIsReversed:
                n = -n
            return ("P", round(n.X, 3), round(n.Y, 3), round(n.Z, 3), round(pl.DistanceTo(rg.Point3d.Origin) if False else (n * rg.Vector3d(pl.Origin)), 3))
        c, cy2 = f.TryGetCylinder(0.001)
        if c:
            return ("C", round(cy2.Center.X, 2), round(cy2.Center.Y, 2), round(cy2.Radius, 2))
        return ("O", f.FaceIndex)
    for f in out.Faces:
        adj = [g for g in f.AdjacentFaces() if g > f.FaceIndex]
        for g in adj:
            if key(f) == key(out.Faces[g]):
                lines.append("same %d %d %s" % (f.FaceIndex, g, key(f)))
        lines.append("%d %s" % (f.FaceIndex, key(f)))
    open(LOG + ".faces", "w").write("\n".join(lines))
    return [out]


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
    open(LOG, "w").write("ok: wrote output.step\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
