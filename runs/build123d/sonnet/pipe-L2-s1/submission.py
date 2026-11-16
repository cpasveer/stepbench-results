import json, pathlib, math
import numpy as np
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
PL = TASK["poses"][0]["placements"]
RA = np.array(PL["A"]["rotation"]); OA = np.array(PL["A"]["origin"])
RB = np.array(PL["B"]["rotation"]); OB = np.array(PL["B"]["origin"])

def V(a): return Vector(*[float(x) for x in a])
def loc(w): return RA.T @ (np.array(w) - OA)      # world point -> pipe frame
def dirl(w): return RA.T @ np.array(w)            # world dir -> pipe frame

# Everything is built in the pipe frame (x along pipe axis, z up) and moved to world at the end.
BU = dirl(RB[:, 0]); BV = dirl(RB[:, 1]); BN = dirl(RB[:, 2]); BO = loc(OB)

T_SADDLE = 6.0
PLATE_T = 6.0
HALF_L = 22.25
R_PIPE = 34.0
ANG = math.radians(35)
PHI = math.radians(62)
d = R_PIPE + T_SADDLE
L1 = 38.0          # straight leg of the arm, along the plate normal
SX = 12.0          # where the arm lands on the saddle ridge (pipe-axis coordinate)
SZ = 40.0
HW = 6.5           # half diagonal of the diamond arm section
TAPER = 14.0       # draft on plate sides (deg), keeps the low edge printable

def unit(v): v = np.array(v, float); return v / np.linalg.norm(v)

def saddle():
    r_o = d / math.cos(PHI - ANG)
    s, c = math.sin(PHI), math.cos(PHI)
    pts = [(30*s, 30*c), (r_o*s, r_o*c), (0, d / math.cos(ANG)), (-r_o*s, r_o*c), (-30*s, 30*c)]
    pl = Plane(origin=(-HALF_L, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    with BuildPart() as p:
        with BuildSketch(pl):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        extrude(amount=2 * HALF_L)
    return p.part

RAMP_DEG = 4.0    # underside of the flange leaves the B face at this angle
X_BED = -HALF_L    # print bed plane (print with the pipe axis vertical, x up)

def plate():
    """Mounting plate on B plus a low flange so the plate's lowest edge reaches the print bed."""
    inset = PLATE_T * math.tan(math.radians(14))
    ramp = math.tan(math.radians(RAMP_DEG))
    pts = [(18.0, 0.0), (-17.0, 0.0), (-40.0, 23.0 * ramp), (-40.0, PLATE_T), (18.0 - inset, PLATE_T)]
    pl = Plane(origin=V(BO), x_dir=V(BU), z_dir=V(-BV))     # sketch (u, n): x -> u, y -> n
    # z_dir = -BV gives y_dir = z x x = -BV x BU = BN
    with BuildPart() as p:
        with BuildSketch(Plane(origin=V(BO + BV * 18), x_dir=V(BU), z_dir=V(-BV))):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        extrude(amount=36.0)
    bed = halfspace([X_BED, 0, 0], [1, 0, 0], True)
    return p.part & bed

def halfspace(point, normal, keep_positive):
    n = unit(normal)
    pl = Plane(origin=V(point), z_dir=V(n if keep_positive else -n))
    return pl * Box(600, 600, 300, align=(Align.CENTER, Align.CENTER, Align.MIN))

def pl2loc(u, v, n): return BO + BU * u + BV * v + BN * n

P0 = (-5.0, 3.5, PLATE_T - 1.5)     # arm root, inside the plate (plate coords u, v, n)
S_END = np.array([SX, 0.0, SZ])

ARM_W = 4.6
ARM_EXT = -1.0
TIP_CUT = 1.8
TH1, TH2 = 45.0, 90.0     # outward normals of the arm section, measured from "down" (gravity) in the section plane

def arm(w=ARM_W):
    a = pl2loc(P0[0], P0[1], 0.0)
    d = unit(S_END - a)
    ex = np.array([1.0, 0, 0])
    qu = unit(ex - d * (ex @ d)); qs = np.cross(d, qu)       # qu: "up" (print direction x) projected into the section
    def nrm(th):
        th = math.radians(th); return -qu * math.cos(th) + qs * math.sin(th)
    n1, n2 = nrm(TH1), nrm(TH2)
    M = np.array([n1, n2, d])
    verts = []
    for s1, s2 in [(1, 1), (1, -1), (-1, -1), (-1, 1)]:
        verts.append(np.linalg.solve(M, np.array([s1 * w, s2 * w, 0.0])))
    # blunt the two acute (45 deg) tips of the section so no wall is thinner than the printer can lay down
    m = nrm((TH1 + TH2) / 2.0)
    pts2 = [np.array(v) for v in verts]
    for sign in (1.0, -1.0):
        lim = max(sign * (m @ v) for v in pts2) - TIP_CUT
        out = []
        for i in range(len(pts2)):
            p, q = pts2[i], pts2[(i + 1) % len(pts2)]
            dp, dq = sign * (m @ p) - lim, sign * (m @ q) - lim
            if dp <= 0: out.append(p)
            if dp * dq < 0: out.append(p + (q - p) * (dp / (dp - dq)))
        pts2 = out
    poly = [a + v for v in pts2]
    length = float(np.linalg.norm(S_END - a)) + ARM_EXT
    seg = Solid.extrude(Face(Wire.make_polygon([V(q) for q in poly], close=True)), V(d * length))
    return seg & halfspace(BO + BN * 3.0, BN, True)

def build():
    body = saddle().fuse(plate(), arm()).clean()
    pipe = Rot(0, 90, 0) * Cylinder(R_PIPE, 400)
    body = body - pipe
    for k, b in BOLTS.items():
        p = loc(b["position"]); a = dirl(b["axis"])
        dia = 5.5 if b["size"] == "M5" else 4.5
        pl = Plane(origin=V(p - a * 3), z_dir=V(a))
        body = body - (pl * Cylinder(dia / 2, 40, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return body.clean() if hasattr(body, "clean") else body

def to_world(shape):
    pl = Plane(origin=V(OA), x_dir=V(RA[:, 0]), z_dir=V(RA[:, 2]))
    return shape.moved(Location(pl))

if __name__ == "__main__":
    part = build()
    print("valid", part.is_valid, "solids", len(part.solids()), "vol", part.volume, "faces", len(part.faces()))
    export_step(to_world(part), str(HERE / "output.step"))
    print("wrote output.step")
