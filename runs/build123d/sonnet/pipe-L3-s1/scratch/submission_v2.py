"""Pipe-to-plate bracket.
Pipe side: two arc bars + axial spine hugging the pipe (extruded along the pipe axis,
so the part prints with the pipe axis vertical), pockets round each head.
Plate side: square plate on the mount.  One diamond-section arm joins them.
Writes output.step."""
import json, math, pathlib, os
import numpy as np
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]
ROT = np.array(TASK["poses"][0]["placements"]["A"]["rotation"])

V = lambda a: Vector(*[float(x) for x in a])
O = V([21.0, 136.5, -7.0])
D = V(ROT[:, 0]).normalized()
NB = V(ROT[:, 2]).normalized()
TG = V(ROT[:, 1]).normalized()
RP = 34.0
C = V(FACE["origin"]); N = V(FACE["normal"]).normalized()

# ---- tunables
P = dict(G=4.2, RB=41.0, BARW=14.0, TH0=45.0, SPINE_HW=8.0, HUB_HW=14.0, HUB_AX=14.0,
         POCKET_R=5.0, TB=3.6, PL=17.5, BEAM_S=16.0, HT=42.0, ELBOW_R=16.0, HOLE_A=5.5, HOLE_B=4.5,
         W=(0.0, 0.0, 0.0))
P.update(json.loads(os.environ.get("P_OVR", "{}")))
G, RB, BARW, TH0 = P["G"], P["RB"], P["BARW"], P["TH0"]
TB, PL = P["TB"], P["PL"]

def cylseg(r, p0, p1):
    p0, p1 = Vector(p0), Vector(p1)
    ax = p1 - p0
    return Part(Cylinder(r, ax.length).moved(Plane(origin=(p0 + p1) * 0.5, z_dir=ax.normalized()).location))

def box(sx, sy, sz, origin, z, x):
    return Part(Box(sx, sy, sz).moved(Plane(origin=origin, x_dir=x, z_dir=z).location))

PIPE_PL = Plane(origin=O, x_dir=NB, z_dir=D)   # local x radial bisector, z pipe axis

def sector(th, z0, z1, R0, R1):
    t = math.radians(th)
    pts = [(0, 0), (2 * R1 * math.cos(t), -2 * R1 * math.sin(t)), (3 * R1, 0), (2 * R1 * math.cos(t), 2 * R1 * math.sin(t))]
    with BuildPart() as bp:
        with BuildSketch(PIPE_PL * Plane.XY.offset(z0)):
            Polygon(*pts, align=None)
        extrude(amount=z1 - z0)
    ring = Part(Cylinder(R1, z1 - z0).moved(PIPE_PL.location * Location((0, 0, (z0 + z1) / 2))))
    return bp.part & ring

saddle = None
for ax in (-17.5, 17.5):
    s = sector(TH0, ax - BARW / 2, ax + BARW / 2, RP, RB)
    saddle = s if saddle is None else saddle + s
zmax = 17.5 + BARW / 2
saddle = saddle + sector(math.degrees(math.asin(P["SPINE_HW"] / (RB - 4))), -zmax, zmax, RP, RB)
saddle = saddle + sector(math.degrees(math.asin(P["HUB_HW"] / (RB - 4))), -P["HUB_AX"], P["HUB_AX"], RP, RB)

# ---------------- B plate
E2 = Vector(0, 0, 1)
E1 = E2.cross(N).normalized()
def fpt(u, v, h=0.0):
    return C + E1 * u + E2 * v + N * h
plate = Part(Box(2 * PL, 2 * PL, TB).moved(Plane(origin=C + N * (TB / 2), x_dir=E1, z_dir=N).location))

# ---------------- arm: vertical stalk off the plate + oblique bar, diamond sections
Wv = V(P["W"]).normalized() if sum(abs(x) for x in P["W"]) > 0 else D

HT = P["HT"]
S = fpt(0, 0, HT)
E = O + NB * (RB - 1.0)
F0 = fpt(0, 0, TB - 1.0)
bn = N.cross(E - S).normalized()          # path-plane normal
print("D.binormal", round(D.dot(bn), 3))
h = P["BEAM_S"] * math.sqrt(2) / 2
with BuildLine() as pth:
    FilletPolyline(F0, S, E, radius=P["ELBOW_R"])
with BuildSketch(Plane(origin=F0, x_dir=bn, z_dir=N)) as sk:
    Polygon((0, -h), (h, 0), (0, h), (-h, 0), align=None)
beam = sweep(sk.sketch, path=pth.wire())
beam = Part(beam.wrapped) if not isinstance(beam, Part) else beam
beam = beam & box(400, 400, 400, C + N * (TB + 200), N, E1)

part = saddle + plate + beam

# ---------------- cuts
pipe = Part(Cylinder(RP, 600).moved(Plane(origin=O, z_dir=D).location))
part = part - pipe
for bid in ("A1", "A2", "A3", "A4"):
    b = BOLTS[bid]; p = V(b["position"]); a = V(b["axis"]).normalized()
    part = part - cylseg(P["POCKET_R"], p + a * G, p + a * 45)
    part = part - cylseg(P["HOLE_A"] / 2, p - a * 8, p + a * (G + 8))
for bid in ("B1", "B2", "B3", "B4"):
    b = BOLTS[bid]; p = V(b["position"]); a = V(b["axis"]).normalized()
    part = part - cylseg(P["HOLE_B"] / 2, p - a * 3, p + a * (TB + 8))

sol=sorted(part.solids(), key=lambda x: x.volume)
print('solids', [round(x.volume) for x in sol])
part = Part(Compound(sol[-1:]).wrapped) if len(sol) > 1 else part

def build():
    return part

if __name__ == "__main__":
    s = build()
    print("nsolids", len(s.solids()), "vol", round(s.volume), "valid", s.is_valid)
    export_step(s, str(HERE / os.environ.get("OUT", "output.step")))
