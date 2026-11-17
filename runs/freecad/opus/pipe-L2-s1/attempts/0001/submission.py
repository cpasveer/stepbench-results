"""Pipe saddle bracket for task pipe-L2-s1, modelled with FreeCAD Part.

Concept
-------
* A saddle shell hugs the 68 mm pipe (inner radius = 34 mm, coaxial).  Its
  outside is faceted: one flat land square to each row of M5 bolts (so the
  cap-screw heads sit on true flat faces) joined by a cylindrical crown.
* A 34 x 34 mm flange sits on the B mount, 1.5 mm inside the 38 mm face.
* A curved arm of teardrop section leaves the flange square to it, bends
  and lands square on the saddle crown, between the four M5 bolts and clear
  of every cap-screw head and hex-key swing.
* Print orientation: pipe axis vertical (saddle end on the bed).  In that
  pose the saddle, the flange and every bolt seat are self-supporting; the
  arm's teardrop section keeps its underside at 50 deg; a short skirt under
  the flange, angled 4 deg away from the B face, carries the flange down to
  the bed.
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
RES = POSE["resolved"]
BOLTS = {b["id"]: b for b in RES["bolts"]}
V = App.Vector


def col(rot, i):
    return V(rot[0][i], rot[1][i], rot[2][i])


# ---- frames -------------------------------------------------------------
RA = POSE["placements"]["A"]["rotation"]
OA = V(*POSE["placements"]["A"]["origin"])
EX, EY, EZ = col(RA, 0), col(RA, 1), col(RA, 2)      # EX = pipe axis
RB = POSE["placements"]["B"]["rotation"]
OB = V(*POSE["placements"]["B"]["origin"])
BU, BV, BN = col(RB, 0), col(RB, 1), col(RB, 2)      # BN = B face normal

R_PIPE = 34.0


def rad(phi_deg):
    p = math.radians(phi_deg)
    return EY * math.sin(p) + EZ * math.cos(p)


def pa(x, r, phi):
    """Point in pipe cylindrical coordinates."""
    return OA + EX * x + rad(phi) * r


def pb(u, v, w=0.0):
    """Point in B-face coordinates (w = height in front of the face)."""
    return OB + BU * u + BV * v + BN * w


# ---- parameters -----------------------------------------------------------
X_BED = -23.5          # saddle bottom (print bed), along pipe axis
X_TOP = 23.5
PHI_END = 50.0         # saddle half-angle
PHI_ROW = 35.0         # bolt rows
R_LAND = 41.0          # bolt lands: 7 mm under the M5 heads
R_CROWN = 43.0         # crown between the lands
T_FLANGE = 6.0         # flange thickness = M4 grip
H_FLANGE = 17.0        # flange half size (B face is 19)
SKIRT_DEG = 4.0
ARM_R = 9.0
ARM_X = 0.0
ARM_PHI = 0.0
TEAR = 50.0
N_SEC = 8            # teardrop flank angle from horizontal


def saddle():
    phi_j = PHI_ROW - math.degrees(math.acos(R_LAND / R_CROWN))   # 17.5

    def land_pt(psi, row):
        return rad(psi) * (R_LAND / math.cos(math.radians(psi - row)))

    p_in_r = rad(PHI_END) * R_PIPE
    p_in_l = rad(-PHI_END) * R_PIPE
    p_out_r = land_pt(PHI_END, PHI_ROW)
    p_out_l = land_pt(-PHI_END, -PHI_ROW)
    p_j_r = rad(phi_j) * R_CROWN
    p_j_l = rad(-phi_j) * R_CROWN
    edges = [
        Part.Arc(p_in_l, rad(0) * R_PIPE, p_in_r).toShape(),
        Part.LineSegment(p_in_r, p_out_r).toShape(),
        Part.LineSegment(p_out_r, p_j_r).toShape(),
        Part.Arc(p_j_r, rad(0) * R_CROWN, p_j_l).toShape(),
        Part.LineSegment(p_j_l, p_out_l).toShape(),
        Part.LineSegment(p_out_l, p_in_l).toShape(),
    ]
    face = Part.Face(Part.Wire(edges))
    face.translate(OA + EX * X_BED)
    return face.extrude(EX * (X_TOP - X_BED))


def flange():
    pts = [pb(-H_FLANGE, -H_FLANGE), pb(H_FLANGE, -H_FLANGE),
           pb(H_FLANGE, H_FLANGE), pb(-H_FLANGE, H_FLANGE)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    plate = face.extrude(BN * T_FLANGE)
    # skirt: continues the flange down to the bed, leaning away from B
    ul = -60.0
    t = math.tan(math.radians(SKIRT_DEG))
    sec = [pb(-H_FLANGE, -H_FLANGE, 0.0), pb(-H_FLANGE, -H_FLANGE, T_FLANGE),
           pb(ul, -H_FLANGE, T_FLANGE), pb(ul, -H_FLANGE, (-H_FLANGE - ul) * t)]
    sk = Part.Face(Part.makePolygon(sec + [sec[0]])).extrude(BV * 2 * H_FLANGE)
    return plate.fuse(sk)


def teardrop(center, tangent, down, r):
    down = (down - tangent * down.dot(tangent)).normalize()
    side = tangent.cross(down).normalize()
    a = math.radians(TEAR)
    tip = center + down * (r / math.cos(a))
    pr = center + side * (r * math.sin(a)) + down * (r * math.cos(a))
    pl = center - side * (r * math.sin(a)) + down * (r * math.cos(a))
    top = center - down * r
    return Part.Wire([
        Part.Arc(pr, top, pl).toShape(),
        Part.LineSegment(pl, tip).toShape(),
        Part.LineSegment(tip, pr).toShape(),
    ])


def arm():
    p0 = pb(0, 0, 2.0)
    p1 = pb(0, 0, T_FLANGE + 22.0)
    p3 = pa(ARM_X, 37.0, ARM_PHI)
    p2 = pa(ARM_X, R_CROWN + 22.0, ARM_PHI)
    bz = Part.BezierCurve()
    bz.setPoles([p0, p1, p2, p3])
    secs = []
    for i in range(N_SEC + 1):
        t = i / N_SEC
        c = bz.value(t)
        tg = bz.tangent(t)[0].normalize()
        secs.append(teardrop(c, tg, EX * -1, ARM_R))
    return Part.makeLoft(secs, True, False)


def holes(body):
    for b in BOLTS.values():
        p = V(*b["position"])
        a = V(*b["axis"])
        d = 5.5 if b["size"] == "M5" else 4.5
        body = body.cut(Part.makeCylinder(d / 2, 80, p - a * 20, a))
    return body


def bed_cut(body):
    big = Part.makeBox(400, 400, 400)
    # half-space below the bed plane x = X_BED (pipe axis is "up")
    pl = App.Placement()
    m = App.Matrix(EY.x, EZ.x, EX.x, 0, EY.y, EZ.y, EX.y, 0, EY.z, EZ.z, EX.z, 0)
    pl = App.Placement(m)
    big.Placement = App.Placement(OA + EX * X_BED, pl.Rotation).multiply(
        App.Placement(V(-200, -200, -400), App.Rotation()))
    return body.cut(big)


def build() -> Part.Shape:
    body = saddle().fuse(flange()).fuse(arm())
    body = bed_cut(body)
    body = holes(body)
    return body.removeSplitter()


OUT = os.path.join(HERE, "output.step")
if __name__ in ("__main__", "submission") or True:
    try:
        if os.path.exists(OUT):
            os.remove(OUT)
        shape = build()
        print("solids", len(shape.Solids), "valid", shape.isValid(), "vol", shape.Volume)
        if len(shape.Solids) == 1:
            shape = shape.Solids[0].removeSplitter()
        shape.exportStep(OUT)
        print("wrote", OUT)
    except Exception:
        traceback.print_exc()
        if os.path.exists(OUT):
            os.remove(OUT)
        sys.exit(1)
