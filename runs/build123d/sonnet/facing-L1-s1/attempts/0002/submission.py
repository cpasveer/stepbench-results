"""Your answer to this task.

Write ONE solid to output.step (next to this file) that bolts to both fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely, or model
the part in any CAD program and save it as output.step instead.

    python submission.py                 # writes output.step
    jointbench verify . --by <your name> --tool <the CAD tool you used>

(the brief, README.md, says how to run them.)
"""
import json
import pathlib

from build123d import Box, Part, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]  # [{"part", "origin", "normal"}]
BOLTS = POSE["bolts"]  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]


def build() -> Part:
    import math
    from build123d import Cylinder, Pos, Rot, Align
    ca, cb = FACES[0]["origin"], FACES[1]["origin"]
    zA, zB = ca[2], cb[2]
    ang = math.degrees(math.atan2(-0.8191521213976911, 0.5735765984126814))
    tA, tB = 4.0, 4.5
    MIN = (Align.CENTER, Align.CENTER, Align.MIN)
    plateA = Pos(ca[0], ca[1], zA) * (Rot(0, 0, ang) * Box(70, 70, tA, align=MIN))
    plateB = Pos(cb[0], cb[1], zB - tB) * Cylinder(41, tB, align=MIN)
    mx, my = (ca[0] + cb[0]) / 2, (ca[1] + cb[1]) / 2
    col = Pos(mx, my, zA + tA - 0.5) * Cylinder(11, (zB - tB) - (zA + tA) + 1.0, align=MIN)
    p = plateA + col + plateB
    for b in BOLTS:
        d = 4.5 if b["size"] == "M4" else 5.5
        x, y, z = b["position"]
        z0 = zA - 1 if b["size"] == "M4" else zB - 11
        p = p - Pos(x, y, z0) * Cylinder(d / 2, 12, align=MIN)
    return Part(p.wrapped)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
