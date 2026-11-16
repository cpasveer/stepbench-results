"""Connecting part for flange-L3-s1.

Frame layout (all numbers from task.json):
  A-side: a hexagonal ring of bars through the six M5 bolt pads plus one spoke
          to the centre; the arm rises from the centre of face A.
  B-side: an H of bars through the four M6 bolt pads, centred on the arm's end.
  arm:    a 13 x 13 mm square prism joining the two faces, cut by both mounting
          planes so its end faces lie exactly on A and B.
Every bolt has a 5 mm (A) / 5 mm (B) thick pad around its hole, so the seat sits
flat on the fixed face and the head bears on the pad top.

    python submission.py                 # writes output.step
"""
import json
import math
import pathlib

from build123d import Align, Box, Cylinder, Face, Location, Plane, Solid, Vector, Wire, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}

Z = (0.0, 0.0, 1.0)

# ---- dimensions (mm) -------------------------------------------------------
PAD_A_R = 5.0      # pad radius around M5 bolt (head radius 4.25)
HOLE_A_D = 5.5     # clearance for M5 (5.3-5.8)
PAD_B_R = 5.5      # pad radius around M6 bolt (head radius 5.0)
HOLE_B_D = 6.6     # clearance for M6 (6.4-7.0)
T_A = 5.0          # plate thickness under the M5 heads (grip 3.75-25)
T_B = 5.0          # plate thickness under the M6 heads (grip 4.5-25)
BAR_W = 4.5        # width of the bars joining the pads
ARM = 13.0         # square arm section


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def face_frame(part):
    """Plane with x along the face's horizontal, y = world Z, z = outward normal
    (the direction from the fixed part into the space my part occupies)."""
    f = FACES[part]
    n = unit(tuple(f["normal"]))
    t = unit(cross(Z, n))
    origin = tuple(f["origin"])
    plane = Plane(origin=origin, x_dir=t, z_dir=n)
    return plane, origin, t, n


planeA, cA, tA, nA = face_frame("A")
planeB, cB, tB, nB = face_frame("B")


def bar(p, q, w, h):
    """Rectangular bar of width w from 2-D point p to q, extruded 0..h in z."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    hw = w / 2
    pts = [
        Vector(p[0] + nx * hw, p[1] + ny * hw, 0),
        Vector(q[0] + nx * hw, q[1] + ny * hw, 0),
        Vector(q[0] - nx * hw, q[1] - ny * hw, 0),
        Vector(p[0] - nx * hw, p[1] - ny * hw, 0),
    ]
    face = Face(Wire.make_polygon(pts))
    return Solid.extrude(face, Vector(0, 0, h))


def pad(c, r, h):
    return Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((c[0], c[1], 0))
    )


def hole(c, d, h):
    return Cylinder(d / 2, h + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((c[0], c[1], -1))
    )


# ---- A side: local x = along face, y = world Z, z = toward B ------------------
r30 = 30.0
A_BOLTS = [(30.0, 0.0), (15.0, 30 * math.sqrt(3) / 2), (-15.0, 30 * math.sqrt(3) / 2),
           (-30.0, 0.0), (-15.0, -30 * math.sqrt(3) / 2), (15.0, -30 * math.sqrt(3) / 2)]


def build_hub_A():
    hub = None
    for p in A_BOLTS:
        s = pad(p, PAD_A_R, T_A)
        hub = s if hub is None else hub + s
    for i in range(len(A_BOLTS)):
        hub = hub + bar(A_BOLTS[i], A_BOLTS[(i + 1) % len(A_BOLTS)], BAR_W, T_A)
    hub = hub + bar((0.0, 0.0), A_BOLTS[0], BAR_W, T_A)  # spoke to the arm root
    for p in A_BOLTS:
        hub = hub - hole(p, HOLE_A_D, T_A)
    return hub


# ---- B side: local x = along face, y = world Z, z = toward A -----------------
B_BOLTS = [(-15.0, -40.0), (15.0, -40.0), (-15.0, 40.0), (15.0, 40.0)]


def build_hub_B():
    hub = None
    for p in B_BOLTS:
        s = pad(p, PAD_B_R, T_B)
        hub = s if hub is None else hub + s
    hub = hub + bar((-15.0, -40.0), (-15.0, 40.0), BAR_W, T_B)
    hub = hub + bar((15.0, -40.0), (15.0, 40.0), BAR_W, T_B)
    hub = hub + bar((-15.0, 0.0), (15.0, 0.0), BAR_W, T_B)
    for p in B_BOLTS:
        hub = hub - hole(p, HOLE_B_D, T_B)
    return hub


def build_arm():
    d = unit(tuple(b - a for a, b in zip(cA, cB)))
    mid = tuple((a + b) / 2 for a, b in zip(cA, cB))
    xa = unit(tuple(t - dot(tA, d) * dd for t, dd in zip(tA, d)))
    length = 180.0
    prism = Box(ARM, ARM, length).moved(Plane(origin=mid, x_dir=xa, z_dir=d).location)
    half_a = Box(400, 400, 400, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(planeA.location)
    half_b = Box(400, 400, 400, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(planeB.location)
    return prism & half_a & half_b


def build():
    hub_a = build_hub_A().moved(planeA.location)
    hub_b = build_hub_B().moved(planeB.location)
    arm = build_arm()
    part = (hub_a + hub_b + arm).clean()
    return part


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "solids", len(part.solids()), "volume", round(part.volume, 1))
