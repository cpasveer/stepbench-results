"""Connecting bracket between pipe A (saddle) and square mount B (flat pad).

Saddle: cylindrical seat R34 on the 68 mm pipe, 8 mm thick, 4x M5 radial bolts.
Pad: 34 x 34 x 8 mm flat seat on the B face, 4x M4 through holes on 25 mm square.
Web: tapered ruled loft from the pad to the saddle crown, kept clear of the
head swing zones (bolt heads sit on the outside of the pad and saddle).
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
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
PL = TASK["poses"][0]["placements"]
V = App.Vector


def placement_matrix(rot, origin):
    """Local -> world: world = origin + rot @ local (rot rows as in task.json)."""
    r, o = rot, origin
    return App.Matrix(r[0][0], r[0][1], r[0][2], o[0],
                      r[1][0], r[1][1], r[1][2], o[1],
                      r[2][0], r[2][1], r[2][2], o[2],
                      0, 0, 0, 1)


MA = placement_matrix(PL["A"]["rotation"], PL["A"]["origin"])  # pipe frame: x = pipe axis
MB = placement_matrix(PL["B"]["rotation"], PL["B"]["origin"])  # mount frame: z = face normal


def to_a(x, y, z):
    return MA.multVec(V(x, y, z))


def to_b(x, y, z):
    return MB.multVec(V(x, y, z))


def dir_b(x, y, z):
    """Direction in the mount frame (rotation only, no translation)."""
    return MB.multVec(V(x, y, z)) - MB.multVec(V(0, 0, 0))


# ---- parameters (mm) ----
R_SEAT = 34.0          # pipe outside radius
SADDLE_T = 8.0         # saddle wall, grip for M5 (3.75..25)
R_OUT = R_SEAT + SADDLE_T
SADDLE_X = 26.0        # half length along pipe axis
SADDLE_TH = 50.0       # half arc, degrees from the bolt-side crown
PAD_SIZE = 34.0        # 2 mm inside the 38 mm mount face
PAD_T = 8.0            # grip for M4 (3..25)
HOLE_A = 5.5           # M5 clearance (5.3..5.8)
HOLE_B = 4.5           # M4 clearance (4.3..4.8)


def saddle():
    """Cylindrical seat on the pipe, bolt side only (theta from +z, |theta|<=50 deg)."""
    ring = Part.makeCylinder(R_OUT, 2 * SADDLE_X, V(-SADDLE_X, 0, 0), V(1, 0, 0)).cut(
        Part.makeCylinder(R_SEAT, 2 * SADDLE_X, V(-SADDLE_X, 0, 0), V(1, 0, 0)))
    r = 60.0
    t = math.radians(SADDLE_TH)
    pts = [V(-30, 0, 0),
           V(-30, -r * math.sin(t), r * math.cos(t)),
           V(-30, 0, r),
           V(-30, r * math.sin(t), r * math.cos(t)),
           V(-30, 0, 0)]
    wedge = Part.Face(Part.makePolygon(pts)).extrude(V(60, 0, 0))
    return ring.common(wedge)


def pad():
    """Flat seat on the B face: local z 0..PAD_T, z=0 on the face, +z toward the pipe."""
    return Part.makeBox(PAD_SIZE, PAD_SIZE, PAD_T, V(-PAD_SIZE / 2, -PAD_SIZE / 2, 0))


def web():
    """Ruled loft from a rectangle on the pad's outer face to a rectangle inside the saddle wall."""
    top = [to_b(sx * 7, sy * 7, PAD_T) for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
    # saddle end: centred on the crown, inside the wall (radius 34..42)
    et = (0.0, 0.945, -0.32)
    bot = []
    for sx, su in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        x = sx * 6.0
        y = su * 3.0 * et[1]
        z = 38.0 + su * 3.0 * et[2]
        bot.append(to_a(x, y, z))
    w1 = Part.makePolygon(top + [top[0]])
    w2 = Part.makePolygon(bot + [bot[0]])
    return Part.makeLoft([w1, w2], True, True)


def bolt_holes_world():
    cuts = []
    for bid in ["A1", "A2", "A3", "A4"]:
        b = BOLTS[bid]
        p, ax = V(*b["position"]), V(*b["axis"])
        cuts.append(Part.makeCylinder(HOLE_A / 2, 40, p - ax * 20, ax))
    return cuts


def mount_holes_world():
    cuts = []
    for x, y in [(-12.5, -12.5), (12.5, -12.5), (12.5, 12.5), (-12.5, 12.5)]:
        cuts.append(Part.makeCylinder(HOLE_B / 2, PAD_T + 4, to_b(x, y, -2), dir_b(0, 0, 1)))
    return cuts


SPOT_Z = 7.8           # flat head seat under each M5 head (just under the 8 mm grip)
SPOT_R = 5.0           # spot-face radius: clears the 4.75 mm head zone with the curved outside


def spot_faces_world():
    """Flat spot under each pipe bolt head: the curved saddle outside dips below
    the head seat, so the material around the head is lowered to a flat seat."""
    cuts = []
    for bid in ["A1", "A2", "A3", "A4"]:
        b = BOLTS[bid]
        p, ax = V(*b["position"]), V(*b["axis"])
        cuts.append(Part.makeCylinder(SPOT_R, 10, p + ax * SPOT_Z, ax))
    return cuts


def build() -> Part.Shape:
    sad = saddle().transformShape(MA)
    for c in spot_faces_world():
        sad = sad.cut(c)
    pd = pad().transformShape(MB)
    body = sad.fuse(pd).fuse(web())
    for c in bolt_holes_world() + mount_holes_world():
        body = body.cut(c)
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
