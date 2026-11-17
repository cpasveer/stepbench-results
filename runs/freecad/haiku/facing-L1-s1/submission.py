"""Connecting part for facing-L1-s1: a bracket between mount A (square, 4x M4)
below and mount B (round, 3x M5) above.

Layout (world Z):
  A face z=-94, A-plate sits on it, z -94 .. -86 (grip 8 mm for the M4 bolts,
        heads sit on top of the plate at z -86 .. -82).
  Column Ø28 runs from the A-plate up to the B-plate, kept away from the
        key-swing sectors around the bolt heads.
  B-plate hangs from the B face z=-25.5, z -33.5 .. -25.5 (grip 8 mm for the
        M5 bolts, heads sit below the plate at z -38.5 .. -33.5).

Plates are built in each mount's local frame (holes_local from task.json),
then moved to world with the mount's placement rotation and origin.
"""
import json
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
POSE = TASK["poses"][0]
PL = POSE["placements"]
RES = POSE["resolved"]
V = App.Vector

PLATE_T = 8.0          # plate thickness = grip for both mounts (3..25 and 3.75..25)
A_HALF = 35.0          # A-plate half size: 1.5 mm inside the 73 mm face
A_CORNER_R = 5.0       # corner arcs centred on the bolt holes
B_R = 40.0             # B-plate radius: 1.25 mm inside the 84.5 mm face (r 42.25)
COL_R = 14.0           # column radius
CLR_M4 = 4.5           # clearance hole (4.3-4.8)
CLR_M5 = 5.5           # clearance hole (5.3-5.8)


def placement_matrix(name):
    """World = origin + R * local, from task.json placements."""
    origin = PL[name]["origin"]
    R = PL[name]["rotation"]
    m = App.Matrix(
        R[0][0], R[0][1], R[0][2], origin[0],
        R[1][0], R[1][1], R[1][2], origin[1],
        R[2][0], R[2][1], R[2][2], origin[2],
        0, 0, 0, 1,
    )
    return m


def rounded_square_plate(half, r, t):
    box = Part.makeBox(2 * half, 2 * half, t, V(-half, -half, 0))
    edges = [e for e in box.Edges
             if abs(e.Vertexes[0].X - e.Vertexes[-1].X) < 1e-9
             and abs(e.Vertexes[0].Y - e.Vertexes[-1].Y) < 1e-9]
    return box.makeFillet(r, edges)


def build() -> Part.Shape:
    bolts = {b["id"]: b for b in RES["bolts"]}
    a_holes_local = [(-30.0, -30.0), (30.0, -30.0), (30.0, 30.0), (-30.0, 30.0)]
    b_holes_local = [(35.0, 0.0), (-17.5, 30.310889132), (-17.5, -30.310889132)]

    # ---- A plate, local frame (local z = 0 on the face, +z away from A)
    a_plate = rounded_square_plate(A_HALF, A_CORNER_R, PLATE_T)
    for x, y in a_holes_local:
        hole = Part.makeCylinder(CLR_M4 / 2, PLATE_T + 2, V(x, y, -1), V(0, 0, 1))
        a_plate = a_plate.cut(hole)
    a_world = a_plate.transformShape(placement_matrix("A"))

    # ---- B plate, local frame (local z = 0 on the face, +z into the part)
    b_plate = Part.makeCylinder(B_R, PLATE_T, V(0, 0, 0), V(0, 0, 1))
    for x, y in b_holes_local:
        hole = Part.makeCylinder(CLR_M5 / 2, PLATE_T + 2, V(x, y, -1), V(0, 0, 1))
        b_plate = b_plate.cut(hole)
    b_world = b_plate.transformShape(placement_matrix("B"))

    # ---- column between the plates, on the midpoint of the two mount centres
    oa = PL["A"]["origin"]
    ob = PL["B"]["origin"]
    xm = 0.5 * (oa[0] + ob[0])
    ym = 0.5 * (oa[1] + ob[1])
    z_bot = oa[2] + PLATE_T           # top of A plate
    z_top = ob[2] - PLATE_T           # bottom of B plate
    column = Part.makeCylinder(COL_R, z_top - z_bot + 0.0, V(xm, ym, z_bot), V(0, 0, 1))

    body = a_world.fuse(column).fuse(b_world)
    body = body.removeSplitter()
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "valid", shape.isValid())
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
