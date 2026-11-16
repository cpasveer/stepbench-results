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
def loc(w):  # world -> pipe-local coords
    return RA.T @ (np.array(w) - OA)
def dirl(w): return RA.T @ np.array(w)

# everything is built in the pipe frame (x along the pipe axis, z up), moved to world at the end
BU = dirl(RB[:, 0]); BV = dirl(RB[:, 1]); BN = dirl(RB[:, 2]); BO = loc(OB)

T_SADDLE = 6.0     # radial thickness under the head faces
PLATE_T = 6.0
HALF_L = 25.0      # half length of the saddle along the pipe
R_PIPE = 34.0
ANG = math.radians(35)
d = R_PIPE + T_SADDLE

def saddle():
    wall = 29.5
    vw = (d - wall * math.sin(ANG)) / math.cos(ANG)
    pts = [(-wall, 10), (-wall, vw), (0, d / math.cos(ANG)), (wall, vw), (wall, 10)]
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
            fillet(*[v for v in sketch.vertices()], radius=3) if False else None
        extrude(amount=PLATE_T)
        edges_z = p.edges().filter_by(Axis(pl.origin, pl.z_dir))
        fillet(edges_z, radius=3)
    return p.part

X1 = 12.0
QU = 0.0
QV = 0.0
def riser(w=12.0, h=14.0):
    S = np.array([X1, 0.0, 40.0])
    Q = BO + BN * 4.5 + BU * QU + BV * QV
    D = Q - S
    L = np.linalg.norm(D); D = D / L
    e1 = np.array([1.0, 0, 0]); e1 = e1 - D * (e1 @ D); e1 /= np.linalg.norm(e1)
    pl = Plane(origin=V(S), x_dir=V(e1), z_dir=V(D))
    with BuildPart() as p:
        with BuildSketch(pl):
            Rectangle(w, h)
        extrude(amount=L)
    return p.part, D

def build():
    body = saddle() + plate()
    r, D = riser()
    body = body + r
    pipe = Pos(0, 0, 0) * (Rot(0, 90, 0) * Cylinder(R_PIPE, 400))
    body = body - pipe
    for k, b in BOLTS.items():
        p = loc(b["position"]); a = dirl(b["axis"])
        dia = 5.5 if b["size"] == "M5" else 4.5
        pl = Plane(origin=V(p - a * 3), z_dir=V(a))
        body = body - (pl * Cylinder(dia / 2, 40, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return body

def to_world(shape):
    pl = Plane(origin=V(OA), x_dir=V(RA[:, 0]), z_dir=V(RA[:, 2]))
    return shape.moved(Location(pl))

if __name__ == "__main__":
    part = build()
    print("D.n", None)
    print("valid", part.is_valid, "solids", len(part.solids()), "vol", part.volume)
    # head clearance interference test (pipe frame)
    for k, b in BOLTS.items():
        p = loc(b["position"]); a = dirl(b["axis"])
        hd = b["head_diameter"] + 1.0
        pl = Plane(origin=V(p + a * (-0.0)), z_dir=V(a))
        # head zone starts at grip end: find by max along axis inside part -> use probe: head zone from 0..hh+1 above end
    w = to_world(part)
    export_step(w, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", w.bounding_box())
