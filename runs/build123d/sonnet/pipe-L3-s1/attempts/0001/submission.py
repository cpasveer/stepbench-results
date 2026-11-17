"""Pipe-to-plate bracket: H-shaped saddle on the pipe, a frame on the square mount,
joined by one round stalk.  Writes output.step."""
import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]

V = lambda a: Vector(*a)
O = V([21.0, 136.5, -7.0])
D = V([0.25, 0.59, 0.77]).normalized()
RP = 34.0
C = V(FACE["origin"]); N = V(FACE["normal"]).normalized()

# ---- tunables
G = 4.2            # pipe-side grip (thickness under head)
RO = RP + G + 0.6  # saddle outer radius
BARW = 12.0        # saddle bar width
TH0 = 48.0         # arc half-angle
SPINE_HW = 6.0
TB = 3.6           # plate thickness (grip on B)
PL = 17.5          # plate half size
BEAM_D = 13.5
HOLE_A, HOLE_B = 5.5, 4.5

a1 = V(BOLTS["A1"]["axis"]); a3 = V(BOLTS["A3"]["axis"])
NB = (a1 + a3).normalized()          # pipe-radial direction at the bisector
NB = (NB - D * NB.dot(D)).normalized()
TG = D.cross(NB)                      # tangential

def cyl(r, h, origin, axis):
    return Pos(origin) * Rotation(*[0,0,0]) * (Plane(origin=(0,0,0), z_dir=axis).location * Cylinder(r, h)) \
        if False else Part(Cylinder(r, h).moved(Plane(origin=origin, z_dir=axis).location))

def cylseg(r, p0, p1):
    p0, p1 = Vector(p0), Vector(p1)
    ax = p1 - p0
    mid = (p0 + p1) * 0.5
    return Part(Cylinder(r, ax.length).moved(Plane(origin=mid, z_dir=ax.normalized()).location))

def box(sx, sy, sz, origin, z, x):
    pl = Plane(origin=origin, x_dir=x, z_dir=z)
    return Part(Box(sx, sy, sz).moved(pl.location))

PIPE_PL = Plane(origin=O, x_dir=NB, z_dir=D)   # local x = radial bisector, y = TG-ish, z = pipe axis

def wedge(th, z0, z1, R):
    """sector |angle|<=th (deg) about NB, axial z0..z1, radius<=R, in pipe frame"""
    t = math.radians(th)
    pts = [(0, 0), (2 * R * math.cos(t), 2 * R * math.sin(t)), (2 * R, 0), (2 * R * math.cos(t), -2 * R * math.sin(t))]
    # fan polygon covering sector
    pts = [(0, 0), (2 * R * math.cos(t), -2 * R * math.sin(t)), (3 * R, 0), (2 * R * math.cos(t), 2 * R * math.sin(t))]
    with BuildPart() as bp:
        with BuildSketch(PIPE_PL * Plane.XY.offset(z0)):
            Polygon(*pts, align=None)
        extrude(amount=z1 - z0)
    return bp.part

def ring(z0, z1, th):
    outer = Part(Cylinder(RO, z1 - z0).moved(PIPE_PL.location * Location((0, 0, (z0 + z1) / 2))))
    sec = wedge(th, z0, z1, RO) & outer
    return sec

# ---------------- saddle (H)
saddle = None
for ax in (-17.5, 17.5):
    r = ring(ax - BARW / 2, ax + BARW / 2, TH0)
    saddle = r if saddle is None else saddle + r
saddle = saddle + ring(-17.5 - BARW / 2, 17.5 + BARW / 2, math.degrees(math.asin(SPINE_HW / (RP + 2))))

# ---------------- B plate
E2 = Vector(0, 0, 1)
E1 = E2.cross(N).normalized()    # in-face horizontal
def fpt(u, v, h=0.0):
    return C + E1 * u + E2 * v + N * h

plate_pl = Plane(origin=C, x_dir=E1, z_dir=N)
with BuildPart() as pb:
    with BuildSketch(plate_pl):
        Rectangle(2 * PL, 2 * PL)
        with Locations((0, 1.5)):
            Rectangle(16, 13.0, mode=Mode.SUBTRACT)
    extrude(amount=TB)
plate = pb.part

# ---------------- stalk
F = fpt(-1.3, -11.5, TB)
E = O + NB * (RP + 2.0)
beam = cylseg(BEAM_D / 2, F - (E - F).normalized() * 6, E + (E - F).normalized() * 4)
# keep only above plate top
keep = box(400, 400, 400, C + N * (TB + 200), N, E1)
beam = beam & keep

part = saddle + plate + beam

# ---------------- cuts
pipe = Part(Cylinder(RP, 600).moved(Plane(origin=O, z_dir=D).location))
part = part - pipe
for bid in ("A1", "A2", "A3", "A4"):
    b = BOLTS[bid]; p = V(b["position"]); a = V(b["axis"]).normalized()
    # spot face
    tool = box(16, 14, 60, p + a * (G + 30), a, D.cross(a).normalized() if False else D.cross(a))
    part = part - tool
    part = part - cylseg(HOLE_A / 2, p - a * 8, p + a * (G + 8))
for bid in ("B1", "B2", "B3", "B4"):
    b = BOLTS[bid]; p = V(b["position"]); a = V(b["axis"]).normalized()
    part = part - cylseg(HOLE_B / 2, p - a * 3, p + a * (TB + 8))

part = Part(part.solids().sort_by(SortBy.VOLUME)[-1]) if len(part.solids()) > 1 else part

def build():
    return part

if __name__ == "__main__":
    s = build()
    print("solids", len(s.solids()), "vol", s.volume, "valid", s.is_valid)
    export_step(s, str(HERE / "output.step"))
