#! python3
"""Pipe-to-square-mount bracket for task pipe-L3-s1, modelled with RhinoCommon.

Layout (pipe frame: x = pipe axis, z = saddle mid-normal, y = z cross x):
  * saddle: 5 mm curved shell on the pipe, carrying the four M5 bolts;
  * web: a slab perpendicular to the pipe axis, standing on the -x end of
    the saddle and reaching out to B, clear of every bolt, tool and key;
  * plate: 5 mm flange on B's mounting face, hung from the web by its
    near edge.
The part is meant to print lying on the flat bottom (pipe frame y = -26),
with the saddle standing up as an arc and the web as a vertical wall.
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
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]
TOL = 0.001
LOGLINES = []


def log(*a):
    LOGLINES.append(" ".join(str(x) for x in a))


def V(a):
    return rg.Vector3d(a[0], a[1], a[2])


def P(a):
    return rg.Point3d(a[0], a[1], a[2])


# ---------------------------------------------------------------- frames
pa = POSE["placements"]["A"]
RA = pa["rotation"]
C = P(pa["origin"])
U = rg.Vector3d(RA[0][0], RA[1][0], RA[2][0])   # pipe axis
LY = rg.Vector3d(RA[0][1], RA[1][1], RA[2][1])
LZ = rg.Vector3d(RA[0][2], RA[1][2], RA[2][2])  # saddle mid-normal
PIPE_PLANE = rg.Plane(C, U, LY)                  # its z axis is LZ
TO_WORLD = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, PIPE_PLANE)
TO_LOCAL = rg.Transform.PlaneToPlane(PIPE_PLANE, rg.Plane.WorldXY)

pb = POSE["placements"]["B"]
RB = pb["rotation"]
BC_W = P(pb["origin"])
BX_W = rg.Vector3d(RB[0][0], RB[1][0], RB[2][0])
BY_W = rg.Vector3d(RB[0][1], RB[1][1], RB[2][1])
NB_W = rg.Vector3d(RB[0][2], RB[1][2], RB[2][2])


def loc_pt(p):
    q = rg.Point3d(p)
    q.Transform(TO_LOCAL)
    return q


def loc_vec(v):
    w = rg.Vector3d(v)
    w.Transform(TO_LOCAL)
    return w


BC = loc_pt(BC_W)
BX, BY, NB = loc_vec(BX_W), loc_vec(BY_W), loc_vec(NB_W)


def bpt(bx, by, n):
    """B-frame coordinates -> pipe-local point."""
    return BC + BX * bx + BY * by + NB * n


# ---------------------------------------------------------------- sizes
R_IN, T_SADDLE = 34.0, 5.0
R_OUT = R_IN + T_SADDLE
Y_BED = -26.0            # flat print face
TH_TOP = math.radians(44.0)
X0, X1 = -32.0, 23.0     # saddle extent along the pipe
SLAB_X1 = -6.0           # web extent along the pipe, B side
WALL_BX = -16.8          # web face square to the plate, clear of the near B bolt heads
NOTCH_X = -22.5          # web stays beyond the A1/A3 heads below NOTCH_Z
NOTCH_Z = 52.0
T_PLATE = 5.0
HALF = 17.8              # seat half size (1 mm inside the 38 mm face)
CHAMF = math.radians(30.0)
GAP_B = 1.0              # web stand-off from B's face
RIB_H = 8.0              # rib height above the plate front
RIB_HALF = 8.2           # rib half width (bolt-head clearance at 8.5)
RIB_BX0, RIB_BX1 = -19.0, 8.0


def yz(x, y, z):
    return rg.Point3d(x, y, z)


def cap_extrude(curve, vec):
    srf = rg.Surface.CreateExtrusion(curve, vec)
    b = srf.ToBrep()
    b.Faces.SplitKinkyFaces(0.01, True)
    b = b.CapPlanarHoles(TOL)
    if b is None:
        raise RuntimeError("cap failed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def saddle():
    def pp(r, th):
        return yz(X0, r * math.sin(th), r * math.cos(th))
    th_ob = math.asin(Y_BED / R_OUT)
    th_ib = math.radians(-44.0)
    a_out = rg.Arc(pp(R_OUT, th_ob), pp(R_OUT, 0.5 * (th_ob + TH_TOP)), pp(R_OUT, TH_TOP))
    a_in = rg.Arc(pp(R_IN, TH_TOP), pp(R_IN, 0.5 * (th_ib + TH_TOP)), pp(R_IN, th_ib))
    pin = pp(R_IN, th_ib)
    pdrop = yz(X0, Y_BED, pin.Z)
    pc = rg.PolyCurve()
    pc.Append(a_out)
    pc.Append(rg.Line(pp(R_OUT, TH_TOP), pp(R_IN, TH_TOP)))
    pc.Append(a_in)
    pc.Append(rg.Line(pin, pdrop))
    pc.Append(rg.Line(pdrop, pp(R_OUT, th_ob)))
    return cap_extrude(pc, rg.Vector3d(X1 - X0, 0, 0))


def box_local(plane, x0, x1, y0, y1, z0, z1):
    return rg.Box(plane, rg.Interval(x0, x1), rg.Interval(y0, y1), rg.Interval(z0, z1)).ToBrep()


def web():
    r = 36.5
    arc_pts = [(r * math.sin(math.radians(a)), r * math.cos(math.radians(a))) for a in (20.5, 15, 0, -15, -30)]
    prof = [(-26.0 + 0.0, math.sqrt(r * r - 26.0 ** 2)), (Y_BED, 105.0), (2.5, 105.0), (36.0, 86.0)] + arc_pts
    pts = [yz(X0, y, z) for (y, z) in prof]
    pts.append(pts[0])
    crv = rg.PolylineCurve(pts)
    slab = cap_extrude(crv, rg.Vector3d(SLAB_X1 - X0, 0, 0))
    notch = box_local(rg.Plane.WorldXY, NOTCH_X, 60.0, -60, 80, -10, NOTCH_Z)
    slab = one(rg.Brep.CreateBooleanDifference(slab, notch, TOL), "notch")
    bpl = rg.Plane(BC, BX, BY)
    # stand-off from B's face (n < GAP_B) and clear of the near B bolts (bx > WALL_BX)
    cut = box_local(bpl, -300, 300, -300, 300, -300, GAP_B)
    slab = one(rg.Brep.CreateBooleanDifference(slab, cut, TOL), "standoff")
    cut = box_local(bpl, WALL_BX, 300, -300, 300, -300, 300)
    slab = one(rg.Brep.CreateBooleanDifference(slab, cut, TOL), "wall")
    return slab


def plate():
    # section in (by, n), extruded along bx; +by edge chamfered so it prints
    prof = [(-HALF, 0.0), (HALF, 0.0), (HALF + T_PLATE * math.tan(CHAMF), T_PLATE), (-HALF, T_PLATE)]
    pts = [bpt(-HALF, by, n) for (by, n) in prof]
    pts.append(pts[0])
    return cap_extrude(rg.PolylineCurve(pts), BX * (2 * HALF))


def rib():
    # stiffening rib on the plate front, between the bolt columns; its +by side
    # leans out 30 deg so it prints, the -by side stays square to the plate
    lean = math.tan(math.radians(30.0))
    n0, n1 = T_PLATE - 0.5, T_PLATE + RIB_H
    prof = [(-RIB_HALF, n0), (RIB_HALF - (n1 - n0) * lean, n0), (RIB_HALF, n1), (-RIB_HALF, n1)]
    pts = [bpt(RIB_BX0, by, n) for (by, n) in prof]
    pts.append(pts[0])
    return cap_extrude(rg.PolylineCurve(pts), BX * (RIB_BX1 - RIB_BX0))


def one(res, what):
    if res is None or len(res) != 1:
        raise RuntimeError("boolean %s gave %s pieces" % (what, None if res is None else len(res)))
    return res[0]


def holes():
    out = []
    for b in BOLTS:
        ax = V(b["axis"])
        ax.Unitize()
        lo, hi = b["clearance_diameter_range"]
        d = 0.5 * (lo + hi)
        base = P(b["position"]) - ax * 4.0
        pl = rg.Plane(base, ax)
        cyl = rg.Cylinder(rg.Circle(pl, d / 2.0), 4.0 + 12.0)
        out.append(cyl.ToBrep(True, True))
    return out



def diag(body):
    import Rhino.Geometry as rg
    log("merge-methods", [m for m in dir(rg.Brep) if "Merge" in m], [m for m in dir(rg.Collections.BrepFaceList) if "Merge" in m or "Join" in m])
    info=[]
    for f in body.Faces:
        s=f.UnderlyingSurface()
        ok,pl=f.TryGetPlane(0.001)
        okc,cy=f.TryGetCylinder(0.001)
        a=rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area
        if ok: key=("P",round(pl.Normal.X,3),round(pl.Normal.Y,3),round(pl.Normal.Z,3),round(pl.Origin.X*pl.Normal.X+pl.Origin.Y*pl.Normal.Y+pl.Origin.Z*pl.Normal.Z,2))
        elif okc: key=("C",round(cy.Radius,2))
        else: key=("O",type(s).__name__)
        info.append(key)
        log("face",f.FaceIndex,key,round(a,1),[e for e in f.AdjacentFaces()])


def build():
    parts = [saddle(), web(), plate(), rib()]
    body = one(rg.Brep.CreateBooleanUnion(parts, TOL), "union")
    body.Transform(TO_WORLD)
    body = one(rg.Brep.CreateBooleanDifference([body], holes(), TOL), "holes")
    body.MergeCoplanarFaces(TOL)
    diag(body)
    log("valid", body.IsValid, "solid", body.IsSolid, "faces", body.Faces.Count,
        "volume", round(body.GetVolume(), 1))
    return [body]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    log("LZ check", round((rg.Vector3d.CrossProduct(U, LY) - LZ).Length, 6))
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build():
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(LOGLINES) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n" + "\n".join(LOGLINES))
    if os.path.exists(OUT):
        os.remove(OUT)
