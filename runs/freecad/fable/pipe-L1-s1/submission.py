"""pipe-L1-s1: bracket joining a 68 mm pipe (4x M5, two rows) to a flat 38 mm
square mount (4x M4).

Design: a saddle pad that follows the pipe, a square flange on the mount, and
one straight round strut between them.  The strut leaves the flange between
bolts B1 and B4 so that every screw head and its hex key stay free, and lands
on the saddle between the two bolt rows.  The M5 heads sit on flat
spotfaces rather than on the curved saddle surface.

    freecadcmd submission.py      # writes output.step
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
POSE0 = TASK["poses"][0]
BOLTS = POSE0["resolved"]["bolts"]
V = App.Vector

# ---- design parameters (mm) -------------------------------------------------
PIPE_R = TASK["fixed_parts"][0]["interface"]["params"]["od"] / 2.0
SADDLE_T = 7.0          # saddle wall thickness
SADDLE_HALF_ANGLE = 49  # deg; bolt rows sit at +-35 deg
SADDLE_HALF_LEN = 26.0  # bolts sit at +-17.5 along the pipe
SPOT_D = 12.0           # M5 spotface diameter
SPOT_GRIP = 5.5         # material left under the M5 head
FLANGE = 36.0           # square flange, 1 mm inside the 38 mm face all round
FLANGE_T = 6.0
FLANGE_CORNER_R = 5.0
STRUT_R = 7.0
STRUT_FOOT = (2.0, 4.0)  # strut axis on the mount face, in B's local x, y
STRUT_LAND_ANGLE = -10   # deg round the pipe from the saddle centre; chosen with
                         # STRUT_FOOT so the strut threads between B1 and B4
HOLE_D = {"M5": 5.5, "M4": 4.5}


def frame(name):
    pl = POSE0["placements"][name]
    r, o = pl["rotation"], pl["origin"]
    m = App.Matrix(r[0][0], r[0][1], r[0][2], o[0],
                   r[1][0], r[1][1], r[1][2], o[1],
                   r[2][0], r[2][1], r[2][2], o[2],
                   0, 0, 0, 1)
    return m


MA, MB = frame("A"), frame("B")


def build():
    # --- saddle, in the pipe's frame (axis = local X, pad centred on +Z) ----
    ro = PIPE_R + SADDLE_T
    L = 2 * SADDLE_HALF_LEN
    shell = Part.makeCylinder(ro, L, V(-L / 2, 0, 0), V(1, 0, 0))
    a = math.radians(SADDLE_HALF_ANGLE)
    far = 3 * ro
    tri = Part.Face(Part.makePolygon([
        V(-L, 0, 0), V(-L, -far * math.sin(a), far * math.cos(a)),
        V(-L, far * math.sin(a), far * math.cos(a)), V(-L, 0, 0)]))
    wedge = tri.extrude(V(2 * L, 0, 0))
    saddle = shell.common(wedge)
    saddle.transformShape(MA)

    # --- flange, in the mount's frame (face = local XY, +Z outward) ---------
    h = FLANGE / 2
    flange = Part.makeBox(FLANGE, FLANGE, FLANGE_T, V(-h, -h, 0))
    vert = [e for e in flange.Edges
            if abs(e.Vertexes[0].Point.z - e.Vertexes[1].Point.z) > 1]
    flange = flange.makeFillet(FLANGE_CORNER_R, vert)
    flange.transformShape(MB)

    # --- strut: straight round bar, mount face -> centre of the saddle ------
    foot = MB.multVec(V(STRUT_FOOT[0], STRUT_FOOT[1], 0))
    la = math.radians(STRUT_LAND_ANGLE)
    land = MA.multVec(V(0, PIPE_R * math.sin(la), PIPE_R * math.cos(la)))
    d = land - foot
    length = d.Length
    d.normalize()
    strut = Part.makeCylinder(STRUT_R, length + 24, foot - d * 12, d)

    body = flange.fuse(strut).fuse(saddle)
    build.strut = strut

    # trim what pokes behind the mount face and into the pipe
    behind = Part.makeBox(200, 200, 50, V(-100, -100, -50))
    behind.transformShape(MB)
    pipe = Part.makeCylinder(PIPE_R, 400, V(-200, 0, 0), V(1, 0, 0))
    pipe.transformShape(MA)
    body = body.cut(behind).cut(pipe)

    # --- bolt holes and M5 spotfaces ----------------------------------------
    for b in BOLTS:
        p, ax = V(*b["position"]), V(*b["axis"])
        body = body.cut(Part.makeCylinder(HOLE_D[b["size"]] / 2, 40, p - ax * 5, ax))
        if b["size"] == "M5":
            body = body.cut(Part.makeCylinder(SPOT_D / 2, 4.0, p + ax * SPOT_GRIP, ax))

    body = body.removeSplitter()

    # designer's own check: room round the mount screws and their keys
    for b in BOLTS:
        p, ax = V(*b["position"]), V(*b["axis"])
        seat = FLANGE_T if b["size"] == "M4" else SPOT_GRIP
        hd, hh = b["head_diameter"], b["head_height"]
        head = Part.makeCylinder(hd / 2, hh + 1, p + ax * seat, ax)
        key = Part.makeCylinder(2.0, 26, p + ax * (seat + hh), ax)
        print(b["id"], "strut to head %.2f, to straight key %.2f" % (
            build.strut.distToShape(head)[0], build.strut.distToShape(key)[0]))
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    assert len(shape.Solids) == 1 and shape.isValid(), "not one valid solid"
    shape = shape.Solids[0]
    print("volume %.0f mm3, faces %d" % (shape.Volume, len(shape.Faces)))
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
