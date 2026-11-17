#! python3
"""flange-L3-s1: bracket between round flange A and rectangular block B.

Concept (printed upside down, world +Z on the bed, no supports):
  * A: upright hexagonal plate (30 deg flanks print freely) inside the round face.
  * B: upright rectangular plate with a small lip that carries it up to the bed plane.
  * A closed diamond tube (torsion + bending) runs along B's normal between the
    plates, hung from a thin fin that reaches the bed plane. The tube sits in the
    height band that is free of every bolt/key column.
  * The tube bore is open through the B plate and blind at A.
"""
import json, math, os, traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
BOLTS = POSE["bolts"]
PL = TASK["poses"][0]["placements"]
TOL = 0.001
msgs = []

def V(a): return rg.Vector3d(a[0], a[1], a[2])
def P(a): return rg.Point3d(a[0], a[1], a[2])

OA = P(PL["A"]["origin"]); OB = P(PL["B"]["origin"])
nA = V(POSE["mounting_faces"][0]["normal"]); nB = V(POSE["mounting_faces"][1]["normal"])
Z = rg.Vector3d(0, 0, 1)
uA = rg.Vector3d.CrossProduct(Z, nA); uA.Unitize()      # horizontal in A face
RB = PL["B"]["rotation"]
sB = rg.Vector3d(RB[0][0], RB[1][0], RB[2][0])          # horizontal (46 mm) direction of B face

# ---------------- parameters ----------------
T_A = 6.5          # A plate thickness (grip M5 3.75-25); thick, mostly infill: stiff between bolts and tube
T_B = 7.5          # B plate thickness (grip M6 4.5-25)
D_A = 5.5; D_B = 6.6
R_HEX = 36.0       # hexagon circum-radius (face radius 37.25, 1 mm setback)
ZC = OA.Z          # 5.5
ZBED = ZC + R_HEX * math.sin(math.radians(60))   # top of the part = print bed
B_HALF = 22.0      # B plate half width (face 23)
B_ZLO = OB.Z - 47.0; B_ZHI = OB.Z + 47.0         # -65 .. 29
LIP_FOOT = 2.4
# tube section (p along sB, z world)
ZAPEX = 22.5       # virtual top apex of the tube (kept below B's upper bolt columns)
ZBT = -15.5        # bottom of the tube (above A's lower bolt columns)
HW = 16.0          # tube half width
BF = 4.0           # half width of the flat tube bottom
FLANK = math.radians(40.0)   # flank angle from vertical (self-supporting)
T_FIN = 2.0; T_WALL = 2.0
U_LAND = -8.0      # where the tube centre line meets A's face plane (u)

def inset(poly, t):
    """inward offset of a convex polygon (list of (x,y))"""
    n = len(poly)
    area = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    sg = 1.0 if area > 0 else -1.0
    lines = []
    for i in range(n):
        x0, y0 = poly[i]; x1, y1 = poly[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0; L = math.hypot(dx, dy)
        nx, ny = -dy / L * sg, dx / L * sg          # inward normal
        lines.append((x0 + nx * t, y0 + ny * t, dx, dy))
    out = []
    for i in range(n):
        ax, ay, adx, ady = lines[i - 1]; bx, by, bdx, bdy = lines[i]
        den = adx * bdy - ady * bdx
        k = ((bx - ax) * bdy - (by - ay) * bdx) / den
        out.append((ax + adx * k, ay + ady * k))
    return out

def prism(pts0, pts1):
    n = len(pts0)
    faces = []
    for pts in (pts0, pts1):
        c = rg.Polyline(list(pts) + [pts[0]]).ToNurbsCurve()
        faces.append(rg.Brep.CreatePlanarBreps(c, TOL)[0])
    for i in range(n):
        j = (i + 1) % n
        faces.append(rg.Brep.CreateFromCornerPoints(pts0[i], pts0[j], pts1[j], pts1[i], TOL))
    b = rg.Brep.JoinBreps(faces, TOL)[0]
    if not b.IsSolid: raise RuntimeError("prism not closed")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward: b.Flip()
    return b

def ptA(u, z, w): return rg.Point3d(OA.X, OA.Y, 0) + uA * u + Z * z + nA * w
def ptB(s, z, n): return rg.Point3d(OB.X, OB.Y, 0) + sB * s + Z * z + nB * n

def to_A_plane(p, w):
    """slide p along nB onto plane w=const of A"""
    t = (w - (p - OA) * nA) / (nB * nA)
    return p + nB * t

def cyl(base, axis, r, h):
    pl = rg.Plane(base, axis)
    return rg.Cylinder(rg.Circle(pl, r), h).ToBrep(True, True)

def one(res, what):
    if res is None or len(res) != 1:
        raise RuntimeError("%s gave %s pieces" % (what, None if res is None else len(res)))
    return res[0]

def build():
    # tube centre offset on B so that it lands at U_LAND on A
    def land(s):
        q = to_A_plane(ptB(s, 0, 0), 0.0)
        return (q - OA) * uA
    l0, l1 = land(0.0), land(1.0)
    s0 = (U_LAND - l0) / (l1 - l0)
    msgs.append("s0 = %.3f" % s0)

    # --- A plate: hexagon
    hexuz = [(R_HEX * math.cos(math.radians(a)), ZC + R_HEX * math.sin(math.radians(a))) for a in range(0, 360, 60)]
    hexuz[1] = (hexuz[1][0], ZBED); hexuz[2] = (hexuz[2][0], ZBED)
    plateA = prism([ptA(u, z, 0) for u, z in hexuz], [ptA(u, z, T_A) for u, z in hexuz])

    # --- B plate with lip (section in n,z extruded along s)
    sec = [(0, B_ZLO), (T_B, B_ZLO), (T_B, ZBED), (T_B - LIP_FOOT, ZBED), (0, B_ZHI)]
    plateB = prism([ptB(-B_HALF, z, n) for n, z in sec], [ptB(B_HALF, z, n) for n, z in sec])

    # --- tube + fin, along nB, from inside B plate to inside A plate
    hf = T_FIN / 2.0
    tn = math.tan(FLANK)
    z1 = ZAPEX - hf / tn
    zst = ZAPEX - HW / tn
    zsb = ZBT + (HW - BF) / tn
    outer = [(-hf, ZBED), (hf, ZBED), (hf, z1), (HW, zst), (HW, zsb), (BF, ZBT), (-BF, ZBT), (-HW, zsb), (-HW, zst), (-hf, z1)]
    o_b = [ptB(s0 + p, z, T_B / 2) for p, z in outer]
    o_a = [to_A_plane(q, T_A / 2) for q in o_b]
    tube = prism(o_b, o_a)

    shell = [(0, ZAPEX), (HW, zst), (HW, zsb), (BF, ZBT), (-BF, ZBT), (-HW, zsb), (-HW, zst)]
    ins = inset(shell, T_WALL)
    # flatten the inner apex (no crevice)
    ft = 2.5
    zf = ins[0][1] - ft / tn
    inner = [(ft, zf)] + ins[1:] + [(-ft, zf)]
    msgs.append("inner: " + " ".join("(%.2f,%.2f)" % q for q in inner))
    i_b = [ptB(s0 + p, z, -5.0) for p, z in inner]
    # the bore is blind and ends square just short of the A plate (A plate stays whole)
    n_end = min(((T_A + 1.0) - (ptB(s0 + p, z, 0.0) - OA) * nA) / (nB * nA) for p, z in inner)
    msgs.append("bore depth from B face %.2f" % n_end)
    i_a = [ptB(s0 + p, z, n_end) for p, z in inner]
    void = prism(i_b, i_a)

    body = one(rg.Brep.CreateBooleanUnion([plateA, plateB, tube], TOL), "union")

    cutters = [void]
    for b in BOLTS:
        ax = V(b["axis"]); pos = P(b["position"])
        isA = b["id"].startswith("A")
        d = D_A if isA else D_B
        t = T_A if isA else T_B
        cutters.append(cyl(pos - ax * 1.0, ax, d / 2.0, t + 2.0))
    for c in cutters:
        body = one(rg.Brep.CreateBooleanDifference(body, c, TOL), "difference")
    body.MergeCoplanarFaces(TOL)
    body.Compact()
    if not (body.IsSolid and body.IsValid): raise RuntimeError("body invalid")
    amp = rg.AreaMassProperties.Compute(body); vmp = rg.VolumeMassProperties.Compute(body)
    msgs.append("faces %d edges %d volume %.0f mm3" % (body.Faces.Count, body.Edges.Count, vmp.Volume))
    me = min(e.GetLength() for e in body.Edges)
    msgs.append("shortest edge %.3f" % me)
    # self check: bolt head / tool columns free
    for b in BOLTS:
        ax = V(b["axis"]); pos = P(b["position"])
        t = T_A if b["id"].startswith("A") else T_B
        col = cyl(pos + ax * (t + 0.02), ax, b["head_diameter"] / 2.0 + 0.5, b["head_height"] + 31.0)
        r = rg.Brep.CreateBooleanIntersection(body, col, TOL)
        if r is not None and len(r) > 0:
            msgs.append("COLUMN BLOCKED " + b["id"])
    return [body]

def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.ModelAbsoluteTolerance = TOL
        for brep in build():
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(msgs) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(msgs) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
