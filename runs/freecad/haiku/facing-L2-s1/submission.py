"""Connector between fixed part A (square mount, above) and fixed part B (round mount).

Design (world millimetres, Z up as placed):
  * bottom plate 70 x 70 x 8 sits on mount A (seat at z=-94), square turned to A's
    rotation; four M4 clearance holes (4.5) through it, heads sit on its top face.
  * central body: solid cylinder R28 from the bottom plate up to mount B's face,
    bored R20 from the B side (6 mm floor left above the bottom plate) to save weight.
  * top plate: three 8 mm thick bosses (R5) under the three M5 bolts, joined to the
    body by 6 mm wide tabs. Seat on mount B at z=-25.5; M5 clearance holes (5.5)
    through the bosses, heads hang below them.
  * every head and its key sweep lies clear of the body; every bearing face is
    either the bottom plate top (faces up) or a small boss underside (short bridge),
    so the part prints Z-up with no support.

Run from this folder:  freecadcmd submission.py   (writes output.step)
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
POSE = TASK["poses"][0]
RES = POSE["resolved"]
FACES = RES["mounting_faces"]
BOLTS = {b["id"]: b for b in RES["bolts"]}
PLACE = POSE["placements"]
V = App.Vector


def build() -> Part.Shape:
    A = FACES[0]["origin"]  # [-126.0, 127.5, -94.0]
    B = FACES[1]["origin"]  # [-124.18, 135.11, -25.5]
    zA = A[2]
    zB = B[2]

    # rotation of mount A (local -> world): turn the bottom plate to match
    rotA = PLACE["A"]["rotation"]
    thetaA = math.degrees(math.atan2(rotA[1][0], rotA[0][0]))

    T_A = 8.0            # bottom plate; grip under the M4 heads
    T_B = 8.0            # top bosses; grip under the M5 heads
    PLATE = 70.0         # inside the 73 mm A face with 1.5 mm to spare
    R_BODY = 28.0        # central body radius (7 mm outside the M5 head axis, fully over the bottom plate)
    R_BORE = 20.0        # bore from the B side
    BORE_FLOOR = zA + T_A + 6.0
    PAD_R = 4.25         # boss radius = M5 head radius (bearing ring just under the head)
    TAB_W = 4.0          # tab width joining bosses to the body
    D_A = 4.5            # M4 clearance (4.3-4.8)
    D_B = 5.5            # M5 clearance (5.3-5.8)

    z_top_bot = zB - T_B  # underside of the B bosses / top plate

    A_pos = [BOLTS[k]["position"] for k in ("A1", "A2", "A3", "A4")]
    B_pos = [BOLTS[k]["position"] for k in ("B1", "B2", "B3")]
    bx, by = B[0], B[1]

    # bottom plate on mount A
    plate = Part.makeBox(PLATE, PLATE, T_A, V(-PLATE / 2, -PLATE / 2, 0))
    plate.rotate(V(0, 0, 0), V(0, 0, 1), thetaA)
    plate.translate(V(A[0], A[1], zA))

    # central body from the bottom plate up to the B seat
    body = Part.makeCylinder(R_BODY, zB - (zA + T_A) + 1.0, V(bx, by, zA + T_A - 1.0), V(0, 0, 1))

    shape = plate.fuse(body)

    # top bosses and tabs
    for p in B_pos:
        boss = Part.makeCylinder(PAD_R, T_B, V(p[0], p[1], z_top_bot), V(0, 0, 1))
        dx, dy = p[0] - bx, p[1] - by
        L = math.hypot(dx, dy)
        phi = math.degrees(math.atan2(dy, dx))
        tab = Part.makeBox(L, TAB_W, T_B, V(0, -TAB_W / 2, 0))
        tab.rotate(V(0, 0, 0), V(0, 0, 1), phi)
        tab.translate(V(bx, by, z_top_bot))
        shape = shape.fuse(boss).fuse(tab)

    shape = shape.removeSplitter()

    # bore from the B side, leaving a floor above the bottom plate
    bore = Part.makeCylinder(R_BORE, (zB + 2.0) - BORE_FLOOR, V(bx, by, BORE_FLOOR), V(0, 0, 1))
    shape = shape.cut(bore)

    # clearance holes: M4 through the bottom plate, M5 through the bosses
    for p in A_pos:
        h = Part.makeCylinder(D_A / 2, T_A + 4.0, V(p[0], p[1], zA - 2.0), V(0, 0, 1))
        shape = shape.cut(h)
    for p in B_pos:
        h = Part.makeCylinder(D_B / 2, T_B + 4.0, V(p[0], p[1], z_top_bot - 2.0), V(0, 0, 1))
        shape = shape.cut(h)

    return shape.removeSplitter()


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
