"""Facing connector between mount A (below: square face, 4x M4) and mount B
(above: round face, 3x M5).

Concept
-------
* A thin-walled vertical tube carries the load from face to face.
* Every bolt gets a solid lobe that runs out from the tube to a pad round
  the bolt.  The B lobes hang under mount B; their undersides slope at 50 deg
  so the part prints A-face down with no support.  The A lobes stand on the
  A face with the same slope on top, capped flat.
* Each bolt head sits in a slot open to the outside: the slot ceiling is a
  short bridge between two 2.5 mm side walls (B), and the hex key reaches in
  and swings outward, clear of the part.
* Pads are clipped 1 mm inside each mounting face's outline.
"""
import json
import math
import pathlib

from build123d import (Box, Cylinder, Location, Part, Plane, Polygon, Pos,
                       Rot, export_step, extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

ZA = FACES["A"]["origin"][2]          # A face, normal +z
ZB = FACES["B"]["origin"][2]          # B face, normal -z
CA = FACES["A"]["origin"][:2]
CB = FACES["B"]["origin"][:2]
H = ZB - ZA
C = ((CA[0] + CB[0]) / 2, (CA[1] + CB[1]) / 2)   # tube axis, between the faces

R_OUT, WALL = 20.0, 1.6              # tube
K_B = math.tan(math.radians(50))     # B lobe underside: 50 deg, prints unsupported
K_A = math.tan(math.radians(28))     # A lobe top: free to be shallow
W_ROOT = 4.5                         # lobe half width where it meets the tube
SEAT_INSET = 1.2                     # seat kept this far inside face outlines

SPEC = {  # hole dia, grip, pocket radius (head + 0.5), lobe half width
    "M4": dict(hole=4.5, grip=4.0, pocket_r=4.0, half_w=6.5, reach=9.0),
    "M5": dict(head_d=8.5, hole=5.5, grip=5.0, pocket_r=4.75, half_w=7.25, reach=8.0),
}

# square A outline, along the bolt pattern sides
_a = BOLTS[0]["position"], BOLTS[1]["position"]
A_ANG = math.degrees(math.atan2(_a[1][1] - _a[0][1], _a[1][0] - _a[0][0]))
A_HALF = TASK["fixed_parts"][0]["interface"]["face_size"][0] / 2
B_RAD = TASK["fixed_parts"][1]["interface"]["face_size"][0] / 2


def frame(b):
    px, py, _ = b["position"]
    dx, dy = px - C[0], py - C[1]
    return math.hypot(dx, dy), math.degrees(math.atan2(dy, dx))


def lobe(b):
    s = SPEC[b["size"]]
    D, ang = frame(b)
    g, w = s["grip"], s["half_w"]
    up = b["axis"][2] > 0            # an A bolt: lobe stands on face A
    s_end = D + s["reach"]
    if up:
        # flat pad at grip height round the head, rising inward 1 mm clear of
        # the head clearance ring
        s1 = D - s["pocket_r"] - 1.0
        pts = [(0, 0), (s_end, 0), (s_end, g), (s1, g), (0, g + K_A * s1)]
    else:
        # underside at 50 deg; at the pad it is deeper than the head pocket
        s_out = s_end + 1.0
        pts = [(0, 0), (s_end, 0), (s_end, g + K_B * (s_out - s_end)),
               (0, g + K_B * s_out)]
    side = extrude(Plane.XZ * Polygon(*pts, align=None), amount=w + 1, both=True)
    s0 = R_OUT - WALL - 1
    plan = Polygon((s0, -W_ROOT), (D, -w), (s_end, -w), (s_end, w), (D, w),
                   (s0, W_ROOT), align=None)
    side &= Pos(0, 0, -100) * extrude(plan, amount=200)
    if up:
        loc = Location((C[0], C[1], ZA)) * Rot(0, 0, ang)
    else:  # flip z' downward; the lobe is symmetric in t
        loc = Location((C[0], C[1], ZB)) * Rot(0, 0, ang) * Rot(180, 0, 0)
    return loc * side


def pocket(b):
    """Hole; for B also the head pocket (head + 0.5 mm, 30 mm tool path) with
    a slot open to the outside whose roof rises at 50 deg past the head."""
    s = SPEC[b["size"]]
    D, ang = frame(b)
    g, r = s["grip"], s["pocket_r"]
    cut = Pos(D, 0, g / 2) * Cylinder(s["hole"] / 2, g + 2)
    if b["axis"][2] > 0:
        return Location((C[0], C[1], ZA)) * Rot(0, 0, ang) * cut
    L = 37.0
    s_h = D + s["head_d"] / 2         # flat roof covers the whole head ring
    cut += Pos(D, 0, g + L / 2) * Cylinder(r, L)
    roof = Polygon((D, g), (s_h, g), (s_h + (g - 1) / K_B, 1), (D + 25, 1),
                   (D + 25, g + L), (D, g + L), align=None)
    cut += extrude(Plane.XZ * roof, amount=r, both=True)
    return Location((C[0], C[1], ZB)) * Rot(0, 0, ang) * Rot(180, 0, 0) * cut


def build() -> Part:
    zm = ZA + H / 2
    a_clip = (Pos(CA[0], CA[1], ZA + 50) * Rot(0, 0, A_ANG)
              * Box(2 * (A_HALF - SEAT_INSET), 2 * (A_HALF - SEAT_INSET), 100))
    b_clip = Pos(CB[0], CB[1], ZB - 50) * Cylinder(B_RAD - SEAT_INSET, 100)
    # tube seam turned to 45 deg, clear of every lobe
    part = Pos(C[0], C[1], zm) * Rot(0, 0, 45) * Cylinder(R_OUT, H)
    for b in BOLTS:
        clip = a_clip if b["axis"][2] > 0 else b_clip
        part += lobe(b) & clip
    part -= Pos(C[0], C[1], zm) * Rot(0, 0, 45) * Cylinder(R_OUT - WALL, H + 2)
    for b in BOLTS:
        part -= pocket(b)
    return part.clean()


if __name__ == "__main__":
    p = build()
    print("solids", len(p.solids()), "volume", round(p.volume, 1), p.is_valid)
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
