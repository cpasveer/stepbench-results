"""Pipe-to-plate bracket (v3).
Square plate on the mount, one horizontal-in-print rhombic arm, and an H-shaped
saddle on the pipe with spot-face pockets round every head.  Designed to print
with the plate's top edge on the bed (world +Z down) and no support.
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
TG = V(ROT[:, 1]).normalized()
NB = V(ROT[:, 2]).normalized()
RP = 34.0
C = V(FACE["origin"]); N = V(FACE["normal"]).normalized()
E2 = Vector(0, 0, 1)
E1 = E2.cross(N).normalized()
WUP = Vector(0, 0, -1)       # print direction (up)

P = dict(G=4.2, RB=41.0, BARW=14.0, TH0=48.0, SPINE_HW=8.0, POCKET_R=5.0,
         TB=3.6, PL=17.5, HOLE_A=5.5, HOLE_B=4.5,
         ARM_H=18.0, ARM_V=24.0, ARM_U0=27.0, BED=32.0, WALL_T=8.0, WING_U=40.0, WING_V0=4.0,
         TILT=8.0, END_AX=29.0, FLANK_AX=-9.0, MINUS_END=-24.5,
         LAND=(22.0, 32.0, 37.0))
P.update(json.loads(os.environ.get("P_OVR", "{}")))
G, RB, BARW, TH0, TB, PL = P["G"], P["RB"], P["BARW"], P["TH0"], P["TB"], P["PL"]

def cylseg(r, p0, p1):
    p0, p1 = Vector(p0), Vector(p1)
    ax = p1 - p0
    return Part(Cylinder(r, ax.length).moved(Plane(origin=(p0 + p1) * 0.5, z_dir=ax.normalized()).location))

def box(sx, sy, sz, origin, z, x):
    return Part(Box(sx, sy, sz).moved(Plane(origin=origin, x_dir=x, z_dir=z).location))

def halfspace(point, normal, keep_neg=True, size=600):
    """solid occupying (p-point).normal <= 0 (keep_neg) """
    nrm = Vector(normal).normalized()
    cen = point - nrm * (size / 2) if keep_neg else point + nrm * (size / 2)
    x = nrm.cross(Vector(0.3, 0.5, 0.8)).normalized()
    return box(size, size, size, cen, nrm, x)

PIPE_PL = Plane(origin=O, x_dir=NB, z_dir=D)

def sector(th, z0, z1, R0, R1):
    t = math.radians(th)
    pts = [(0, 0), (2 * R1 * math.cos(t), -2 * R1 * math.sin(t)), (3 * R1, 0), (2 * R1 * math.cos(t), 2 * R1 * math.sin(t))]
    with BuildPart() as bp:
        with BuildSketch(PIPE_PL * Plane.XY.offset(z0)):
            Polygon(*pts, align=None)
        extrude(amount=z1 - z0)
    ring = Part(Cylinder(R1, z1 - z0).moved(PIPE_PL.location * Location((0, 0, (z0 + z1) / 2))))
    return bp.part & ring

# ---------------- saddle: bar at -17.5, bar at +17.5, spine
E2h = (E2 - D * D.dot(E2)).normalized()          # in-plane direction of "world up" seen from the pipe
dl = math.radians(P["TILT"])
NU = (D * math.cos(dl) - E2h * math.sin(dl)).normalized()   # tilted normal for +D facing faces
def tilt_plane_pt(ax_at_ref):
    # plane through the point on the pipe axis at axial=ax_at_ref displaced to s=ref along E2h
    return O + D * ax_at_ref + E2h * 38.0

bar_m = sector(TH0, P["MINUS_END"], -17.5 + BARW / 2 + 10, RP, RB) & halfspace(tilt_plane_pt(P["FLANK_AX"]), NU)
bar_p = sector(TH0, 17.5 - BARW / 2, 17.5 + BARW / 2 + 12, RP, RB)
spine = sector(math.degrees(math.asin(P["SPINE_HW"] / (RB - 4))), P["MINUS_END"], 17.5 + BARW / 2 + 12, RP, RB)
saddle = bar_m + bar_p + spine
saddle = saddle & halfspace(tilt_plane_pt(P["END_AX"]), NU)

# ---------------- plate + base wall (the wall stands on the bed when printing)
def fpt(u, v, h=0.0):
    return C + E1 * u + E2 * v + N * h
BED, WT = P["BED"], P["WALL_T"]
plate = Part(Box(2 * PL, 2 * PL, TB).moved(Plane(origin=C + N * (TB / 2), x_dir=E1, z_dir=N).location))
def slab(u0, u1, v0, v1, h0, h1):
    # box with local axes x=E1, y=E2, z=N   (E1 x E2 = N)
    pl = Plane(origin=fpt((u0 + u1) / 2, (v0 + v1) / 2, (h0 + h1) / 2), x_dir=E1, z_dir=N)
    return Part(Box(u1 - u0, v1 - v0, h1 - h0).moved(pl.location))
# slab A: rear-edge bevel keeps it clear of the face plane (no seat beyond the outline), 33 deg bevel
with BuildPart() as sa:
    with BuildSketch(Plane(origin=fpt(0, 0, 0), x_dir=E2, z_dir=E1)):   # sketch x = v, y = z x x = E1 x E2 = N -> h
        Polygon((17.0, 0.0), (PL, 0.0), (PL + 1.5, 1.0), (BED, 1.0), (BED, 1.0 + WT), (17.0, 1.0 + WT), align=None)
    extrude(amount=PL, both=True)
slabA = sa.part
slabB = slab(PL, P["WING_U"], P["WING_V0"], BED, 1.0, 1.0 + WT)
base = plate + slabA + slabB

# ---------------- arm: straight rhombic bar, keel climbs >= 27 deg when printed with -Z up
lu, lax, lR = P["LAND"]
th = math.radians(lax)
T = O + D * lu + (NB * math.cos(th) + TG * math.sin(th)) * lR
tq = T - C
print("landing u,v,n", round(tq.dot(E1), 1), round(tq.dot(E2), 1), round(tq.dot(N), 1))
hh, vh = P["ARM_H"] / 2, P["ARM_V"] / 2
R0 = fpt(P["ARM_U0"], BED - vh - 0.3, 1.0 + WT / 2)
ax = (T - R0).normalized()
yv = (E2 - ax * ax.dot(E2)).normalized()
xv = yv.cross(ax).normalized()
print("keel slope deg", round(math.degrees(math.asin(abs(ax.dot(E2)))), 1), "(printed with -E2 up)")
R0b = R0 - ax * 20.0
with BuildPart() as ab:
    with BuildSketch(Plane(origin=R0b, x_dir=xv, z_dir=ax)):
        Polygon((0, -vh), (hh, 0), (0, vh), (-hh, 0), align=None)
    extrude(amount=(T - R0b).length)
# clip flush with the wall's rear face and the bed plane so the bar roots cleanly in the wall
arm = ab.part & halfspace(C + N * 1.0, -N) & halfspace(C + E2 * BED, E2)

part = saddle + base + arm

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

sol = sorted(part.solids(), key=lambda x: x.volume)
print("solids", [round(x.volume) for x in sol])
if len(sol) > 1:
    part = Part(Compound(sol[-1:]).wrapped)

def build():
    return part

if __name__ == "__main__":
    s = build()
    print("nsolids", len(s.solids()), "vol", round(s.volume), "valid", s.is_valid)
    export_step(s, str(HERE / os.environ.get("OUT", "output.step")))
