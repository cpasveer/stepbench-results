"""Connector between round pipe A (4x M5 radial) and flat mount B (4x M4 on 25 mm square).

Design: two saddle straps bolted to the pipe (one per bolt pair), a 38-mm-class
flange plate bolted to B, and two 10x10 mm bridging beams joining the plate to
the straps. Everything is in world millimetres. Output: output.step.
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
HEAD_D = {"M5": 8.5, "M4": 7.0}
HEAD_H = {"M5": 5.0, "M4": 4.0}
CLEAR_D = {"M5": 5.5, "M4": 4.5}

# pipe A
P0 = V(21.0, 136.5, -7.0)
PIPE_R = 34.0
AX = V(0.2487956429525063, 0.5900653629615055, 0.7680648380709507)
AX.normalize()

# mount B
CB = V(*TASK["poses"][0]["placements"]["B"]["origin"])
NB = V(*POSE["mounting_faces"][0]["normal"])
NB.normalize()
UB = V(0.9214414224885092, -0.3885173161164798, 0.0)
UB.normalize()
ZB = V(0, 0, 1)

# design values (mm)
STRAP_TOP = PIPE_R + 5.0      # flat top of each pipe strap: grip 5 under the M5 head
STRAP_HALF_W = 8.0            # tangential half width of a strap
STRAP_HALF_L = 23.5           # axial half length of a strap (bolts at +-17.5)
PLATE_HALF = 17.0             # flange plate 34 x 34
PLATE_T = 4.0                 # flange plate thickness (grip under M4 head)
BEAM = 10.0                   # bridging beam square section


def frame_box(origin, ex, ey, ez, l, w, h):
    """Box with local x along ex (length l), y along ey (w), z along ez (h)."""
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
    """Square-section prism of side w from point p to point q."""
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


def cyl(base, direction, r, h):
    return Part.makeCylinder(r, h, base, direction)


def build():
    axis_of = {k: V(*BOLT[k]["axis"]) for k in BOLT}
    pipe = cyl(P0 - AX * 150, AX, PIPE_R, 300)

    # --- saddle straps on the pipe, one per radial bolt pair (A1/A2 and A3/A4)
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
        strap_pts.append(P0 + r * (PIPE_R + 2.0))

    # --- flange plate on the B face, standing on the face, growing toward the pipe
    plate_origin = CB - UB * PLATE_HALF - ZB * PLATE_HALF
    plate = frame_box(plate_origin, UB, NB, ZB, 2 * PLATE_HALF, PLATE_T, 2 * PLATE_HALF)
    body = body.fuse(plate)

    # --- a post stands on the plate, clear of the B bolt heads and tool access,
    #     then two beams run from the top of the post to the two straps
    post_base = CB - UB * 4.0 - ZB * 4.0
    post_top = post_base + NB * 44.0
    body = body.fuse(bar(post_base + NB * 2.0, post_top, BEAM))
    for sp in strap_pts:
        body = body.fuse(bar(post_top, sp, BEAM))

    # --- clearance holes for every bolt
    for bid, b in BOLT.items():
        pos = V(*b["position"])
        ax = V(*b["axis"])
        if b["size"] == "M5":
            body = body.cut(cyl(pos - ax * 3.0, ax, 2.75, 60))
        else:
            body = body.cut(cyl(pos - ax * 3.0, ax, 2.25, 25))

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
