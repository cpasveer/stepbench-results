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
HALF_L = 23.0
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

def plate():
    pl = Plane(origin=V(BO), x_dir=V(BU), z_dir=V(BN))
    with BuildPart() as p:
        with BuildSketch(pl):
            Rectangle(36, 36)
        extrude(amount=PLATE_T, taper=TAPER)
        fillet(p.edges().filter_by(lambda e: abs(unit([e.end_point().X-e.start_point().X, e.end_point().Y-e.start_point().Y, e.end_point().Z-e.start_point().Z]) @ BN) > 0.5), radius=3)
    return p.part

def halfspace(point, normal, keep_positive):
    n = unit(normal)
    pl = Plane(origin=V(point), z_dir=V(n if keep_positive else -n))
    return pl * Box(600, 600, 300, align=(Align.CENTER, Align.CENTER, Align.MIN))

def pl2loc(u, v, n): return BO + BU * u + BV * v + BN * n

P0 = (-5.0, 3.5, PLATE_T - 1.5)     # arm root, inside the plate (plate coords u, v, n)
S_END = np.array([SX, 0.0, SZ])

def arm(hw=HW):
    a = pl2loc(*P0)
    d = unit(S_END - a)
    ex = np.array([1.0, 0, 0])
    p1 = unit(ex - d * (ex @ d)); p2 = np.cross(d, p1)     # p1: diamond diagonal along gravity (print along x)
    poly = [a + p1 * hw, a + p2 * hw, a - p1 * hw, a - p2 * hw]
    length = float(np.linalg.norm(S_END - a)) + 12
    seg = Solid.extrude(Face(Wire.make_polygon([V(q) for q in poly], close=True)), V(d * length))
    return Part(seg.wrapped)

def build():
    body = saddle() + plate() + arm()
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
