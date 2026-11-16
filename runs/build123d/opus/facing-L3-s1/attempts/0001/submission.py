"""Facing connector between mount A (below, square 4xM4) and mount B (above,
round 3xM5).

Concept: a thin-walled vertical tube carries the load between the two faces.
Each bolt gets a solid "lobe" running from the tube out to a pad round the
bolt.  The B lobes hang under mount B with a 50 deg underside (no support
needed when printed A-face down); the bolt heads sit in pockets cut up into
those lobes.  The A lobes are mirror images standing on the A face.
"""
import json
import math
import pathlib

from build123d import (Axis, Box, Cylinder, Location, Part, Plane, Polygon,
                       export_step, extrude, Pos, Rot)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

ZA = FACES["A"]["origin"][2]          # A face (normal +z)
ZB = FACES["B"]["origin"][2]          # B face (normal -z)
CA = FACES["A"]["origin"][:2]
CB = FACES["B"]["origin"][:2]
C = ((CA[0] + CB[0]) / 2, (CA[1] + CB[1]) / 2)   # tube axis

R_OUT, WALL = 20.0, 2.4
K = math.tan(math.radians(50))       # lobe slope (50 deg from horizontal)

SPEC = {  # size: hole dia, grip, head radius, pocket radius, pad radius
    "M4": dict(hole=4.5, grip=4.0, head_r=3.5, pocket_r=4.0, pad_r=5.25),
    "M5": dict(hole=5.5, grip=5.0, head_r=4.25, pocket_r=4.75, pad_r=5.75),
}


def lobe(b):
    s = SPEC[b["size"]]
    px, py, _ = b["position"]
    dx, dy = px - C[0], py - C[1]
    D = math.hypot(dx, dy)
    ang = math.degrees(math.atan2(dy, dx))
    rp, g = s["pad_r"], s["grip"]
    s_out = D + s["head_r"]          # slope reaches grip depth at head edge
    s_end = D + rp + 2
    up = b["axis"][2] > 0            # A bolt (lobe stands on A)
    # side profile in the (s, z') plane, z' measured from the face into the part
    prof = Polygon((0, 0), (s_end, 0), (s_end, g - K * (s_end - s_out)),
                   (0, g + K * s_out), align=None)
    side = extrude(Plane.XZ * prof, amount=rp + 1, both=True)
    plan = Pos(D / 2, 0, 0) * Box(D, 2 * rp, 200) + Pos(D, 0, 0) * Cylinder(rp, 200)
    body = side & plan
    # head pocket: head + clearance + 30 mm tool access
    body -= Pos(D, 0, g + 18.5) * Cylinder(s["pocket_r"], 37)
    if up:
        loc = Location((C[0], C[1], ZA)) * Rot(0, 0, ang)
    else:  # flip z' downward (the lobe is symmetric in t, so the y flip is harmless)
        loc = Location((C[0], C[1], ZB)) * Rot(0, 0, ang) * Rot(180, 0, 0)
    return loc * body


def build() -> Part:
    H = ZB - ZA
    part = Pos(C[0], C[1], ZA + H / 2) * Cylinder(R_OUT, H)
    for b in BOLTS:
        part += lobe(b)
    part -= Pos(C[0], C[1], ZA + H / 2) * Cylinder(R_OUT - WALL, H + 2)
    for b in BOLTS:
        px, py, pz = b["position"]
        part -= Pos(px, py, (ZA + ZB) / 2) * Cylinder(SPEC[b["size"]]["hole"] / 2, H + 2)
    return part.clean()


if __name__ == "__main__":
    p = build()
    print("solids", len(p.solids()), "volume", round(p.volume, 1), p.is_valid)
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
