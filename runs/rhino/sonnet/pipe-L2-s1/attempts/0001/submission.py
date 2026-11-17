#! python3
import json, os, math, traceback
HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")
import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
BOLTS = {b["id"]: b for b in POSE["resolved"]["bolts"]}
FACE = POSE["resolved"]["mounting_faces"][0]
TOL = 0.001
msgs = []

def V(a): return rg.Vector3d(a[0], a[1], a[2])
def P(a): return rg.Point3d(a[0], a[1], a[2])
def unit(v):
    v = rg.Vector3d(v); v.Unitize(); return v
def dot(a, b): return a * b
def cross(a, b): return rg.Vector3d.CrossProduct(a, b)

# ---- pipe
PO = rg.Point3d(21.0, 136.5, -7.0)
PD = unit(V([0.25, 0.59, 0.77]))
PR = 34.0
R1 = unit(V(BOLTS["A1"]["axis"]))
R2 = unit(V(BOLTS["A3"]["axis"]))
BIS = unit(R1 + R2)
cosa = BIS * R1
G = 6.0          # grip / pad thickness on pipe
RT = PR + G
S_EDGE = 9.0     # pad extends this far past bolt axis (away side)
L_AX = 25.5      # half length along pipe axis

def away(ra, rb):
    t = rb - (rb * ra) * ra
    t.Unitize()
    return -t

def pipe_tent():
    t1 = away(R1, R2); t2 = away(R2, R1)
    pts3 = [rg.Point3d(0, 0, 0) + 0.0 * BIS,
            P(R1 * RT + t1 * S_EDGE), P(BIS * (RT / cosa)), P(R2 * RT + t2 * S_EDGE)]
    pts = [PO - PD * L_AX + (rg.Vector3d(p) if not isinstance(p, rg.Vector3d) else p) for p in pts3]
    pl = rg.Polyline(pts + [pts[0]])
    crv = pl.ToPolylineCurve()
    srf = rg.Surface.CreateExtrusion(crv, PD * (2 * L_AX))
    brep = srf.ToBrep().CapPlanarHoles(TOL)
    return brep

# ---- B frame
BO = P(FACE["origin"]); BN = unit(V(FACE["normal"]))
BW = rg.Vector3d(0, 0, 1)
BU = unit(cross(BW, BN)) * -1.0   # chosen so that (BW, BU, BN) is right-handed
if cross(BW, BU) * BN < 0: BU = -BU
def bw(x, y, z):  # x along BU, y along BW, z along BN
    return BO + BU * x + BW * y + BN * z

def b_plate():
    pl = rg.Plane(BO, BW, BU)   # x=BW, y=BU, z=BN
    box = rg.Box(pl, rg.Interval(-17, 17), rg.Interval(-17, 17), rg.Interval(0, 6.0))
    return box.ToBrep()

def strut():
    bx = (-6, 6); by = (-17, -8); z0 = 2.0
    base = [bw(bx[0], by[0], z0), bw(bx[1], by[0], z0), bw(bx[1], by[1], z0), bw(bx[0], by[1], z0)]
    top_c = PO + BIS * 40.0
    base_c = bw(0, -12.5, z0)
    Vv = top_c - base_c
    corners = base + [p + Vv for p in base]
    msgs.append("strut dir %s len %.1f" % (unit(Vv), Vv.Length))
    for p in corners[4:]:
        v = p - PO
        ax = v * PD
        rad = v - PD * ax
        msgs.append("top corner rho_b=%.1f rho_r1=%.1f rho_r2=%.1f axial=%.1f radius=%.1f" % (rad * BIS, rad * R1, rad * R2, ax, rad.Length))
    return rg.Brep.CreateFromBox(corners)

def cyl(center, axis, r, h0, h1):
    pl = rg.Plane(center + axis * h0, axis)
    c = rg.Cylinder(rg.Circle(pl, r), h1 - h0)
    return c.ToBrep(True, True)

def build():
    tent = pipe_tent(); plate = b_plate(); st = strut()
    for nm, bb in (("tent", tent), ("plate", plate), ("strut", st)):
        if bb.GetVolume() < 0: bb.Flip()
        msgs.append("%s vol %.1f solid %s" % (nm, bb.GetVolume(), bb.IsSolid))
    u = rg.Brep.CreateBooleanUnion([tent, plate, st], TOL)
    if not u: raise RuntimeError("union failed")
    body = u[0]
    tools = [cyl(PO - PD * 200, PD, PR, 0, 400)]
    for k in ("A1", "A2", "A3", "A4"):
        b = BOLTS[k]; ax = unit(V(b["axis"])); p = P(b["position"])
        tools.append(cyl(p, ax, 2.75, -12, G + 4))
    for k in ("B1", "B2", "B3", "B4"):
        b = BOLTS[k]; ax = unit(V(b["axis"])); p = P(b["position"])
        tools.append(cyl(p, ax, 2.25, -3, 6 + 4))
    msgs.append("union solid=%s valid=%s vol=%.1f" % (body.IsSolid, body.IsValid, body.GetVolume()))
    res = body
    for t in tools:
        if t.GetVolume() < 0: t.Flip()
    for i, t in enumerate(tools):
        d = rg.Brep.CreateBooleanDifference([res], [t], TOL)
        if not d:
            raise RuntimeError("difference failed at tool %d" % i)
        res = d[0]
        msgs.append("after tool %d: solid=%s vol=%.1f" % (i, res.IsSolid, res.GetVolume()))
    msgs.append("solid=%s volume=%.1f faces=%d" % (res.IsSolid, res.GetVolume(), res.Faces.Count))
    return [res]

def main():
    if os.path.exists(OUT): os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build():
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT): raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(msgs) + "\n")

try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(msgs))
    if os.path.exists(OUT): os.remove(OUT)
