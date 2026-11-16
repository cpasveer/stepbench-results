"""Pipe-saddle to square-mount bracket (task pipe-L3-s1), FreeCAD script.

    freecadcmd submission.py      # writes output.step

Design
------
Everything is modelled in the pipe's own frame (x along the pipe axis, the
four M5 bolts at x = +-17.5, +-35 deg round from local +z) and moved into world
coordinates at the end.

* Saddle: a faceted clamp pad, bore exactly the 68 mm pipe, with one flat
  bolt land per bolt row (square to the bolts, 5 mm under the heads) and a
  flat crown between them.  It is a straight prism along the pipe axis.
* Plate: 5 mm plate on mount B, seat 34 x 34 (1 mm+ inside B's 38 mm face).
* Web: a 14 mm wide rib running from the saddle crown, between the bolt
  heads of B, to the plate, plus a thin skirt under the plate's lower edge.
  Its side walls are all parallel to the pipe axis.

Print orientation: the saddle end face (x = +24) on the bed, pipe axis
vertical.  Then the saddle bore, its lands and all web walls are vertical,
the plate's faces lean at 50 deg, and the plate's lowest edge sits on the
web/skirt, so it prints without support.
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
V = App.Vector

# ---------------------------------------------------------------- frames ---
RA = POSE["placements"]["A"]["rotation"]
CA = V(*POSE["placements"]["A"]["origin"])
RB = POSE["placements"]["B"]["rotation"]
CB = V(*POSE["placements"]["B"]["origin"])


def col(R, i):
    return V(R[0][i], R[1][i], R[2][i])


AX, AY, AZ = col(RA, 0), col(RA, 1), col(RA, 2)


def to_local_pt(p):
    d = V(*p) - CA
    return V(d.dot(AX), d.dot(AY), d.dot(AZ))


def to_local_vec(w):
    w = V(*w)
    return V(w.dot(AX), w.dot(AY), w.dot(AZ))


# mount B frame, expressed in the pipe frame
BC = to_local_pt(CB)
BU = to_local_vec(col(RB, 0))
BV = to_local_vec(col(RB, 1))
BN = to_local_vec(col(RB, 2))  # out of B's face, towards the pipe

BOLTS = []
for b in POSE["resolved"]["bolts"]:
    BOLTS.append(dict(id=b["id"], p=to_local_pt(b["position"]),
                      a=to_local_vec(b["axis"]), hd=b["head_diameter"],
                      hh=b["head_height"]))

# ------------------------------------------------------------ parameters ---
R_PIPE = 34.0
LAND = 38.5           # bolt land distance from pipe axis -> 4.5 mm under M5 heads
CROWN = 41.0          # flat crown between the two bolt rows
HALF_ANG = 46.0       # saddle half angle (bolt rows at 35 deg)
X_END = 23.0          # saddle runs x = -23 .. +23 (bolts at +-17.5)
ROW_ANG = 35.0
PLATE_T = 5.0
SEAT = 17.0           # seat half width (B face is 19)
WEB_HALF = 7.0        # web half width between B's bolt heads (v = +-12.5)
U_JOIN = -8.0         # web top meets the plate's back face at u = U_JOIN
X_CROWN = 8.0         # ... and the saddle crown at x = X_CROWN
D_A, D_B = 5.5, 4.5   # clearance holes M5 / M4

BIG = 2000.0


def halfspace(point, normal):
    """Solid {p : (p - point).normal <= 0} (a big box)."""
    n = V(normal).normalize()
    box = Part.makeBox(BIG, BIG, BIG, V(-BIG / 2, -BIG / 2, -BIG))
    box.Placement = App.Placement(point, App.Rotation(V(0, 0, 1), n))
    return box


def bpt(u, v, n):
    return BC + BU * u + BV * v + BN * n


# (t, v) coordinates of the plane normal to the pipe axis:
# v = B's v coordinate, t along S (perpendicular to v, towards B's +u).
S = V(0, BV.z, -BV.y)
if S.dot(BU) < 0:
    S = -S
S.normalize()
BCV = BC.dot(BV)
T_B = BC.dot(S)


def tv(t, v, x=0.0):
    yz = S * t + BV * (v + BCV)
    return V(x, yz.y, yz.z)


# ----------------------------------------------------------------- saddle --
def saddle():
    c, s = math.cos, math.sin
    r = math.radians
    # end rays at +-HALF_ANG meet the bolt lands
    re = LAND / c(r(HALF_ANG - ROW_ANG))
    yc = (LAND - CROWN * c(r(ROW_ANG))) / s(r(ROW_ANG))
    pts = [V(0, 0, 0),
           V(0, -re * s(r(HALF_ANG)), re * c(r(HALF_ANG))),
           V(0, -yc, CROWN),
           V(0, yc, CROWN),
           V(0, re * s(r(HALF_ANG)), re * c(r(HALF_ANG))),
           V(0, 0, 0)]
    pts = [p + V(-X_END, 0, 0) for p in pts]
    face = Part.Face(Part.makePolygon(pts))
    body = face.extrude(V(2 * X_END, 0, 0))
    pipe = Part.makeCylinder(R_PIPE, 400, V(-200, 0, 0), V(1, 0, 0))
    return body.cut(pipe)


# ------------------------------------------------------------------ plate --
T_WALL = T_B + 0.64037208 * SEAT  # vertical (print) end wall through seat edge u=+17


def plate():
    body = Part.makeBox(400, 400, 400, V(-200, -200, -200))
    for pnt, nrm in [(bpt(0, 0, 0), -BN), (bpt(0, 0, PLATE_T), BN),
                     (bpt(-SEAT, 0, 0), -BU),
                     (bpt(0, SEAT, 0), BV), (bpt(0, -SEAT, 0), -BV),
                     (tv(T_WALL, 0), S)]:
        body = body.common(halfspace(pnt, nrm))
    return body


# -------------------------------------------------------------------- web --
def web():
    def at(a_deg, rr):
        a = math.radians(a_deg)
        # (t, v) of a point at angle a (from +t towards +v) round the pipe axis
        return (rr * math.cos(a), -BCV + rr * math.sin(a))

    p1 = at(8, 37.0)
    p2 = at(52, 37.0)
    t_bend = 62.0
    # skirt under the plate's lowest edge: from u0 = 18 on the back face
    t_cb = T_B + 0.64037208 * 18.0 - 0.76806484 * PLATE_T
    w = WEB_HALF
    outline = [p1, (t_bend, -w), (t_cb, -w), (t_cb, -SEAT), (T_WALL, -SEAT),
               (T_WALL, SEAT), (t_cb, SEAT), (t_cb, w), (t_bend, w), p2, p1]
    pts = [tv(t, v, -150.0) for t, v in outline]
    prism = Part.Face(Part.makePolygon(pts)).extrude(V(150.0 + X_END, 0, 0))
    body = prism.common(halfspace(bpt(0, 0, PLATE_T), -BN))
    # top (in print) of the web: plane along v through the plate's back
    # face (u = U_JOIN) and through the saddle crown
    pa = bpt(U_JOIN, 0, PLATE_T)
    pb = tv(30.0, 0, X_CROWN)
    nrm = BV.cross(pb - pa)
    if nrm.x > 0:
        nrm = -nrm
    body = body.common(halfspace(pa, nrm))
    return body


def holes(shape):
    for b in BOLTS:
        d = D_A if b["id"].startswith("A") else D_B
        h = 12.0
        cyl = Part.makeCylinder(d / 2, h + 2, b["p"] - b["a"] * 2, b["a"])
        shape = shape.cut(cyl)
    return shape


def build_local():
    part = saddle().fuse(plate()).fuse(web())
    part = holes(part)
    part = part.removeSplitter()
    return part


def to_world(shape):
    m = App.Matrix(AX.x, AY.x, AZ.x, CA.x,
                   AX.y, AY.y, AZ.y, CA.y,
                   AX.z, AY.z, AZ.z, CA.z,
                   0, 0, 0, 1)
    return shape.transformGeometry(m) if False else shape.transformed(m)


def build() -> Part.Shape:
    return to_world(build_local())


OUT = os.path.join(HERE, "output.step")
if __name__ in ("__main__", "submission"):
    try:
        if os.path.exists(OUT):
            os.remove(OUT)
        shape = build()
        if len(shape.Solids) == 1:
            shape = shape.Solids[0].removeSplitter()
        else:
            print("WARNING: solids =", len(shape.Solids))
        shape.exportStep(OUT)
        print("wrote", OUT, "volume", round(shape.Volume, 1))
    except Exception:
        traceback.print_exc()
        if os.path.exists(OUT):
            os.remove(OUT)
        sys.exit(1)
