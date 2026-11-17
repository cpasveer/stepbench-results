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

P = dict(G=4.2, RB=41.0, BARW=14.0, TH0=45.0, SPINE_HW=8.0, POCKET_R=5.0,
         TB=3.6, PL=17.5, HOLE_A=5.5, HOLE_B=4.5,
         ARM_H=15.0, ARM_V=24.0, ARM_VB=-16.0, ARM_NS=44.0, ELBOW_R=16.0,
         TILT=8.0, END_AX=29.0, FLANK_AX=-9.0, MINUS_END=-24.5,
         LAND=(22.0, 32.0, 37.0), FOOT_N=33.0, FOOT_V=(16.5, 17.9))
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

# ---------------- plate + foot
def fpt(u, v, h=0.0):
    return C + E1 * u + E2 * v + N * h
plate = Part(Box(2 * PL, 2 * PL, TB).moved(Plane(origin=C + N * (TB / 2), x_dir=E1, z_dir=N).location))
fv0, fv1 = P["FOOT_V"]
foot = Part(Box(2 * PL, P["FOOT_N"], fv1 - fv0).moved(
    Plane(origin=fpt(0, (fv0 + fv1) / 2, P["FOOT_N"] / 2), x_dir=E1, z_dir=E2).location))
# Box local y = E2 x E1 = -N, so the foot spans n in [0, FOOT_N]; it lies on the plate's top edge

# ---------------- arm (rhombic section, swept along an elbowed path)
AR = P["ARM_VB"]
lu, lax, lR = P["LAND"]
th = math.radians(lax)
T = O + D * lu + (NB * math.cos(th) + TG * math.sin(th)) * lR
tq = T - C
tu, tv, tn = tq.dot(E1), tq.dot(E2), tq.dot(N)
print("landing u,v,n", round(tu, 1), round(tv, 1), round(tn, 1))
F0 = fpt(0, AR, TB - 0.5)
P1 = fpt(0, AR, P["ARM_NS"])
P2 = fpt(tu, tv, tn)
hh, vh = P["ARM_H"] / 2, P["ARM_V"] / 2
with BuildLine() as pth:
    FilletPolyline(F0, P1, P2, radius=P["ELBOW_R"])
bn = (P1 - F0).cross(P2 - P1).normalized()
with BuildSketch(Plane(origin=F0, x_dir=E1, z_dir=N)) as sk:
    Polygon((0, -vh), (hh, 0), (0, vh), (-hh, 0), align=None)
arm = sweep(sk.sketch, path=pth.wire())
arm = Part(arm.wrapped)
arm = arm & halfspace(C + N * (TB - 0.5), -N)

part = saddle + plate + foot + arm

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
