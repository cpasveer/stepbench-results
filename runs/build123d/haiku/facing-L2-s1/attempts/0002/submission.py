"""Connector between the square mount A (below) and the round mount B (above).

Design: a central hub sits between the two faces. Mount A (4x M4) is carried
by four low arms lying on the bed, so the arms and bosses print flat with no
overhang. Mount B (3x M5) is carried by three bosses on sloped webs, whose
undersides are 45 degrees from the hub. Only the hub stands in the mid gap,
so the L-key sweeps of all seven bolts stay clear.

Only output.step is graded. Run:  python submission.py
"""
import json
import math
import pathlib

from build123d import (
    BuildPart,
    BuildSketch,
    Box,
    Cone,
    Cylinder,
    Plane,
    Polygon,
    Pos,
    Rot,
    export_step,
    extrude,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = {b["id"]: b for b in POSE["bolts"]}

ZA = FACES["A"]["origin"][2]  # mount A face, -94.0 (our part sits above it)
ZB = FACES["B"]["origin"][2]  # mount B face, -25.5 (our part sits below it)
CB = FACES["B"]["origin"][:2]  # centre of the round mount

# --- dimensions (mm) -------------------------------------------------------
GRIP_A = 6.0  # foot thickness under the M4 heads (grip range 3-25)
GRIP_B = 6.0  # foot thickness above the M5 heads (grip range 3.75-25)
HOLE_A = 4.5  # M4 clearance (4.3-4.8)
HOLE_B = 5.5  # M5 clearance (5.3-5.8)
HUB_R = 10.0
ARM_W = 9.0
BOSS_A_R = 5.0  # keeps the A feet 1 mm inside the 73 mm face
BOSS_B_R = 5.5  # keeps the B feet 1 mm inside the 84.5 mm face
WEB_W = 5.0
WEB_END = 31.9  # web ends inside the B boss, clear of the hole
SEAT_R = 4.25  # flat B head seat (M5 head radius), minimises the downward-facing area
CHAMFER_H = 1.6  # steep (52 deg) chamfer from the seat up to the boss rim
WEB_ANGLE = 47.0  # underside of the web, degrees from horizontal


def web(ux, uy, cx, cy):
    """Sloped web from the hub out to one B boss, in the vertical plane through the boss."""
    # local x runs out along u, local y is world z, plane normal is -tangent
    plane = Plane(origin=(cx, cy, 0), x_dir=(ux, uy, 0), z_dir=(uy, -ux, 0))
    zb0 = ZB - GRIP_B  # underside of the boss
    drop = (WEB_END - 5.0) * math.tan(math.radians(WEB_ANGLE))
    pts = [
        (5.0, ZB),
        (WEB_END, ZB),
        (WEB_END, zb0),
        (5.0, zb0 - drop),  # sloped underside running into the hub
    ]
    with BuildPart() as p:
        with BuildSketch(plane):
            Polygon(*pts)
        extrude(amount=WEB_W / 2, both=True)
    return p.part


def boss_b(px, py):
    """B foot: flat seat disc at the bottom, steep chamfer, then the round boss to the face."""
    zb0 = ZB - GRIP_B
    seat = Pos(px, py, zb0 + CHAMFER_H / 2) * Cone(SEAT_R, BOSS_B_R, CHAMFER_H)
    rim = Pos(px, py, (zb0 + CHAMFER_H + ZB) / 2) * Cylinder(BOSS_B_R, ZB - zb0 - CHAMFER_H)
    return seat + rim


def build():
    cbx, cby = CB
    hub_h = ZB - ZA
    body = Pos(cbx, cby, (ZA + ZB) / 2) * Cylinder(HUB_R, hub_h)

    for bid in ("A1", "A2", "A3", "A4"):
        px, py, _ = BOLTS[bid]["position"]
        dx, dy = px - cbx, py - cby
        length = math.hypot(dx, dy)
        ang = math.degrees(math.atan2(dy, dx))
        mx, my = cbx + dx / 2, cby + dy / 2
        arm = Pos(mx, my, ZA + GRIP_A / 2) * Rot(0, 0, ang) * Box(length, ARM_W, GRIP_A)
        boss = Pos(px, py, ZA + GRIP_A / 2) * Cylinder(BOSS_A_R, GRIP_A)
        body = body + arm + boss

    for bid in ("B1", "B2", "B3"):
        px, py, _ = BOLTS[bid]["position"]
        dx, dy = px - cbx, py - cby
        r = math.hypot(dx, dy)
        ux, uy = dx / r, dy / r
        body = body + web(ux, uy, cbx, cby)
        body = body + boss_b(px, py)

    # clearance holes, from each mount face to the head seat
    for bid in ("A1", "A2", "A3", "A4"):
        px, py, _ = BOLTS[bid]["position"]
        body = body - Pos(px, py, ZA + GRIP_A / 2) * Cylinder(HOLE_A / 2, GRIP_A + 4)
    for bid in ("B1", "B2", "B3"):
        px, py, _ = BOLTS[bid]["position"]
        body = body - Pos(px, py, ZB - GRIP_B / 2) * Cylinder(HOLE_B / 2, GRIP_B + 4)

    return body


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
