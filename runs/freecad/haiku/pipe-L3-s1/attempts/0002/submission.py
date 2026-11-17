"""Connector between round pipe A (4x M5 radial bolts) and flat mount B (4x M4 on a 25 mm square).

Design, in world millimetres:
  * two saddle straps, one under each radial bolt pair, whose inner faces follow
    the 68 mm pipe (r = 34) and whose flat tops carry the M5 heads (grip 5 mm);
  * a flange plate 34 x 34 x 4 mm on the B face, bolted with its four M4 bolts;
  * a short hub post on the plate, from which three 10 x 10 mm legs fan out to the
    straps. The legs form a tripod, so the 4.9 N load at B is carried mainly in
    axial stiffness instead of bending. Clearance is kept round every bolt head.
Output: output.step next to this file.
"""
import json
import os
import sys
import traceback

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
V = App.Vector

BOLT = {b["id"]: b for b in POSE["bolts"]}

# pipe A: axis point and unit direction (fixed_A.step, world coordinates)
P0 = V(21.0, 136.5, -7.0)
PIPE_R = 34.0
AX = V(0.2487956429525063, 0.5900653629615055, 0.7680648380709507)
AX.normalize()

# mount B: face centre, outward normal (pointing at the pipe), and in-face axes
CB = V(*TASK["poses"][0]["placements"]["B"]["origin"])
NB = V(*POSE["mounting_faces"][0]["normal"])
NB.normalize()
UB = V(0.9214414224885092, -0.3885173161164798, 0.0)
UB.normalize()
ZB = V(0, 0, 1)

# design values (mm)
STRAP_TOP = PIPE_R + 5.0      # flat strap top under the head: grip 5 mm
STRAP_HALF_W = 8.0            # tangential half width of a strap
STRAP_HALF_L = 23.5           # axial half length (bolts at +-17.5)
STRAP_S = 6.0                 # leg attachment offsets along strap 1
PLATE_HALF = 17.0             # flange 34 x 34
PLATE_T = 4.0                 # flange thickness under the M4 heads
LEG = 10.0                    # leg section (square)
LEG_ROOT_A = (1.0, 8.0)       # leg root on the plate (u, v) for the two strap-1 legs
LEG_ROOT_B = (3.5, -16.0)    # leg root for the strap-3 leg


def frame_box(origin, ex, ey, ez, l, w, h):
    """Box with local x along ex (length l), y along ey (w), z along ez (h). ex, ey, ez right-handed."""
    s = Part.makeBox(l, w, h)
    m = App.Matrix(
        ex.x, ey.x, ez.x, origin.x,
        ex.y, ey.y, ez.y, origin.y,
        ex.z, ey.z, ez.z, origin.z,
        0, 0, 0, 1,
    )
    s.transformShape(m)
    return s


def bar(p, q, w):
    """Square-section prism of side w from point p to point q (flat ends)."""
    d = q - p
    L = d.Length
    ex = V(d)
    ex.normalize()
    helper = V(0, 0, 1) if abs(ex.z) < 0.9 else V(1, 0, 0)
    ey = ex.cross(helper)
    ey.normalize()
    ez = ex.cross(ey)
    origin = p - ey * (w / 2.0) - ez * (w / 2.0)
    return frame_box(origin, ex, ey, ez, L, w, w)


def build():
    axis_of = {k: V(*BOLT[k]["axis"]) for k in BOLT}
    pipe = Part.makeCylinder(PIPE_R, 300, P0 - AX * 150, AX)

    # --- saddle straps: flat top at the head seat, concave inner face on the pipe
    body = None
    strap_pts = []
    for pair in (("A1", "A2"), ("A3", "A4")):
        r = axis_of[pair[0]]
        tg = AX.cross(r)
        tg.normalize()
        origin = P0 - AX * STRAP_HALF_L - tg * STRAP_HALF_W
        blk = frame_box(origin, AX, tg, r, 2 * STRAP_HALF_L, 2 * STRAP_HALF_W, STRAP_TOP)
        strap = blk.cut(pipe)
        body = strap if body is None else body.fuse(strap)
        strap_pts.append((r, P0 + r * 38.0))

    # --- flange plate on the B face, standing on the face toward the pipe
    plate_origin = CB - UB * PLATE_HALF - ZB * PLATE_HALF
    plate = frame_box(plate_origin, UB, NB, ZB, 2 * PLATE_HALF, PLATE_T, 2 * PLATE_HALF)
    body = body.fuse(plate)

    # --- three legs carry the load from the plate to the straps (a tripod).
    #     Two leave the plate between B1/B2's row and B3, reach strap 1 either side
    #     of the bolt pair; one leaves the plate's -v edge and passes outside B1.
    r1, _ = strap_pts[0]
    r3, _ = strap_pts[1]
    root_a = CB + UB * LEG_ROOT_A[0] + ZB * LEG_ROOT_A[1] + NB * 2.0
    root_b = CB + UB * LEG_ROOT_B[0] + ZB * LEG_ROOT_B[1] + NB * 2.0
    legs = [
        (root_a, P0 - AX * STRAP_S + r1 * 38.5),
        (root_a, P0 + AX * STRAP_S + r1 * 38.5),
        (root_b, P0 + r3 * 38.5),
    ]
    for root, tgt in legs:
        body = body.fuse(bar(root, tgt, LEG))

    # --- clearance holes for every bolt
    for bid, b in BOLT.items():
        pos = V(*b["position"])
        ax = V(*b["axis"])
        if b["size"] == "M5":
            body = body.cut(Part.makeCylinder(2.75, 60, pos - ax * 3.0, ax))
        else:
            body = body.cut(Part.makeCylinder(2.25, 25, pos - ax * 3.0, ax))

    # --- nothing may go behind the B face or into the pipe
    behind = frame_box(CB - UB * 100 - ZB * 100 - NB * 60, UB, NB, ZB, 200, 60, 200)
    body = body.cut(behind)
    body = body.cut(pipe)
    return body.removeSplitter()


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
