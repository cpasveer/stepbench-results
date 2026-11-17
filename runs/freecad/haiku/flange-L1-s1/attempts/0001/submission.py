"""Connector between mount A (round, 6x M5) and mount B (rectangular, 4x M6).

Design (built in the frame of mount A: origin at A's centre, z along A's normal):
  * A pad  - disc R35 x 8 thick sitting on mount A, 6 clearance holes on the
             60 mm bolt circle, heads sit on its top face.
  * B pad  - 42 x 92 plate (3 mm corner radius) sitting on mount B, 4 clearance
             holes on the 30 x 80 pattern, heads sit on its outer face.
  * Pillar - one R9 column joining the two pads, placed where it clears every
             bolt head and the key swing of every bolt.
Run:  freecadcmd submission.py   (writes output.step next to this file)
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
POSE = TASK["poses"][0]["placements"]
V = App.Vector

RA = POSE["A"]["rotation"]
OA = POSE["A"]["origin"]
RB = POSE["B"]["rotation"]
OB = POSE["B"]["origin"]


def transpose(M):
    return [[M[j][i] for j in range(3)] for i in range(3)]


def matmul(M, N):
    return [[sum(M[i][k] * N[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat(R, o):
    return App.Matrix(R[0][0], R[0][1], R[0][2], o[0],
                      R[1][0], R[1][1], R[1][2], o[1],
                      R[2][0], R[2][1], R[2][2], o[2],
                      0, 0, 0, 1)


# A-frame -> world, and B-frame -> A-frame (frames are right-handed, columns = axes)
M_WORLD_FROM_A = mat(RA, OA)
_RAt = transpose(RA)
_d = [OB[i] - OA[i] for i in range(3)]
OB_IN_A = [sum(_RAt[i][j] * _d[j] for j in range(3)) for i in range(3)]
M_A_FROM_B = mat(matmul(_RAt, RB), OB_IN_A)

# ---- parameters (mm) ----
T_A = 8.0          # A pad thickness = grip for M5 (range 3.75-25)
R_A = 35.0         # A pad radius (seat must stay 1 mm inside the 37.25 face)
HOLE_A = 5.5       # M5 clearance (5.3-5.8)
T_B = 8.0          # B pad thickness = grip for M6 (range 4.5-25)
B_PAD = (42.0, 92.0, 3.0)  # width x height x corner radius (face is 46 x 96)
HOLE_B = 6.6       # M6 clearance (6.4-7.0)
PILLAR_R = 9.0
PILLAR_XY = (-3.6, -7.3)   # A-frame footprint, clears A bolts and B bolts
PILLAR_TOP_S = 4.0         # pillar stops 4 mm into the B pad

A_BOLTS = [(30.0, 0.0), (15.0, 25.980762114), (-15.0, 25.980762114),
           (-30.0, 0.0), (-15.0, -25.980762114), (15.0, -25.980762114)]
B_BOLTS = [(-15.0, -40.0), (15.0, -40.0), (-15.0, 40.0), (15.0, 40.0)]


def build_in_a() -> Part.Shape:
    # A pad: disc on mount A, z = 0 .. T_A (z is A's outward normal)
    a_pad = Part.makeCylinder(R_A, T_A, V(0, 0, 0), V(0, 0, 1))

    # B pad, built in B's frame (z = B's outward normal, pad at z 0 .. T_B)
    bw, bh, br = B_PAD
    b_pad = Part.makeBox(bw, bh, T_B, V(-bw / 2, -bh / 2, 0))
    vert = [e for e in b_pad.Edges
            if abs(e.Vertexes[0].Z - e.Vertexes[-1].Z) > 1e-6]
    b_pad = b_pad.makeFillet(br, vert)
    b_pad = b_pad.transformShape(M_A_FROM_B)

    # Pillar joining the pads: vertical in A frame, clipped to B-frame s >= 4
    pillar = Part.makeCylinder(PILLAR_R, 120.0,
                               V(PILLAR_XY[0], PILLAR_XY[1], 2.0), V(0, 0, 1))
    slab = Part.makeBox(400, 400, 400, V(-200, -200, PILLAR_TOP_S))
    slab = slab.transformShape(M_A_FROM_B)
    pillar = pillar.common(slab)

    body = a_pad.fuse(pillar).fuse(b_pad)

    # Clearance holes: A holes along A z, B holes along B z
    for x, y in A_BOLTS:
        body = body.cut(Part.makeCylinder(HOLE_A / 2, T_A + 2, V(x, y, -1), V(0, 0, 1)))
    for x, y in B_BOLTS:
        h = Part.makeCylinder(HOLE_B / 2, T_B + 2, V(x, y, -1), V(0, 0, 1))
        body = body.cut(h.transformShape(M_A_FROM_B))
    return body


def build() -> Part.Shape:
    shape = build_in_a()
    shape = shape.transformShape(M_WORLD_FROM_A)
    return shape


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
