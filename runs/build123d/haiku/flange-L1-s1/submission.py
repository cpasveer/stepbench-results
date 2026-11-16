"""Connector between the round flange A and the rectangular block B.

Two seat pads sit flat on the two mounting faces (the pads face each other),
joined by a round spine. Every bolt passes through a clearance hole in its pad
and its head bears on the pad top. Dimensions come from task.json.
"""
import json
import math
import pathlib

from build123d import (BuildPart, BuildSketch, Circle, Plane, RectangleRounded,
                       Solid, Vector, export_step, extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
RES = POSE["resolved"]
BOLTS = {b["id"]: b for b in RES["bolts"]}

# ---- design choices (mm) ----
GRIP = 10.0          # pad thickness = grip under the head (A: 3.75-25, B: 4.5-25)
A_PAD_R = 35.0       # round seat pad, inside the 37.25 radius mount face by >1 mm
B_PAD = (42.0, 90.0, 5.0)  # seat pad outline w x h, corner radius (inside 46x96 by >1 mm)
A_HOLE_D = 5.5       # M5 clearance (5.3-5.8)
B_HOLE_D = 6.6       # M6 clearance (6.4-7.0)
SPINE_R = 10.0


def frame(part):
    """Local frame of a fixed part: origin on its mounting face, z = outward normal."""
    pl = POSE["placements"][part]
    R = pl["rotation"]
    x_dir = [R[i][0] for i in range(3)]
    z_dir = [R[i][2] for i in range(3)]
    return Plane(origin=pl["origin"], x_dir=x_dir, z_dir=z_dir)


def vec(v):
    return Vector(*v)


def build() -> Solid:
    plA, plB = frame("A"), frame("B")
    nA, nB = plA.z_dir, plB.z_dir

    with BuildPart() as pad_a:
        with BuildSketch(plA):
            Circle(A_PAD_R)
        extrude(amount=GRIP)
    with BuildPart() as pad_b:
        with BuildSketch(plB):
            RectangleRounded(B_PAD[0], B_PAD[1], B_PAD[2])
        extrude(amount=GRIP)

    # spine: from inside pad A to inside pad B
    qa = plA.origin + nA * 5.0
    qb = plB.origin + nB * 5.0
    axis = qb - qa
    spine = Solid.make_cylinder(SPINE_R, axis.length, Plane(origin=qa, z_dir=axis.normalized()))

    body = pad_a.part.fuse(spine).fuse(pad_b.part)

    # clearance holes on every bolt axis, through the pad to the head seat
    for bid, b in BOLTS.items():
        d = A_HOLE_D if b["size"] == "M5" else B_HOLE_D
        u = vec(b["axis"]).normalized()
        p = vec(b["position"])
        hole = Solid.make_cylinder(d / 2, GRIP + 3.0, Plane(origin=p - u * 3.0, z_dir=u))
        body = body - hole
    return body.clean()


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(part.volume, 1))
