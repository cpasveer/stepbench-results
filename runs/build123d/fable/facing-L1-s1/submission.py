"""Connector for facing-L1-s1: a spool-type adapter.

Square foot flange on A (4x M4), round head flange on B (3x M5), joined by a
hollow round column with filleted roots.  All bolts sit outside the column so
their hex keys swing freely outward.

    python submission.py    # writes output.step
"""
import json
import math
import pathlib

from build123d import (Axis, Cylinder, Part, Pos, Rot, Box, chamfer, fillet,
                       export_step, GeomType)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

T_A = 8.0          # foot flange thickness (grip of the M4s)
T_B = 8.0          # head flange thickness (grip of the M5s)
SETBACK = 1.5      # flange outline inside the mounting face outline
COL_R = 21.0       # column outer radius
BORE_R = 13.0      # column bore radius (8 mm wall)
ROOT_R = 5.0       # fillet at column roots
D_M4, D_M5 = 4.5, 5.5


def build() -> Part:
    ax, ay, za = FACES["A"]["origin"]
    bx, by, zb = FACES["B"]["origin"]
    a_bolts = [b for b in BOLTS if b["id"].startswith("A")]
    b_bolts = [b for b in BOLTS if b["id"].startswith("B")]

    # orientation of A's square about the vertical, from its bolt pattern
    p2, p3 = a_bolts[1]["position"], a_bolts[2]["position"]
    ang = math.degrees(math.atan2(p3[1] - p2[1], p3[0] - p2[0]))

    side = TASK["fixed_parts"][0]["interface"]["face_size"][0] - 2 * SETBACK
    dia = TASK["fixed_parts"][1]["interface"]["face_size"][0] - 2 * SETBACK

    # foot flange on A
    foot = Pos(ax, ay, za + T_A / 2) * Rot(0, 0, ang) * Box(side, side, T_A)
    foot = fillet(foot.edges().filter_by(Axis.Z), 5.0)
    foot = chamfer(foot.edges().group_by(Axis.Z)[-1], 1.0)

    # head flange on B
    head = Pos(bx, by, zb - T_B / 2) * Cylinder(dia / 2, T_B)
    head = chamfer(head.edges().group_by(Axis.Z)[0], 1.0)

    # column between them, on the mean axis of the two mounts
    cx, cy = (ax + bx) / 2, (ay + by) / 2
    z0, z1 = za + T_A, zb - T_B
    col = Pos(cx, cy, (z0 + z1) / 2) * Cylinder(COL_R, z1 - z0)

    part = foot + col + head
    roots = [e for e in part.edges().filter_by(GeomType.CIRCLE)
             if abs(e.radius - COL_R) < 1e-6]
    part = fillet(roots, ROOT_R)

    # weight-saving bore right through
    part -= Pos(cx, cy, (za + zb) / 2) * Cylinder(BORE_R, zb - za)

    # bolt holes
    for b in a_bolts:
        x, y, _ = b["position"]
        part -= Pos(x, y, za + T_A / 2) * Cylinder(D_M4 / 2, T_A)
    for b in b_bolts:
        x, y, _ = b["position"]
        part -= Pos(x, y, zb - T_B / 2) * Cylinder(D_M5 / 2, T_B)
    return part


if __name__ == "__main__":
    p = build()
    assert len(p.solids()) == 1 and p.is_valid
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(p.volume), "faces", len(p.faces()))
