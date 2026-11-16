"""Hub-and-spoke connector.

A hollow central column stands on A's face and carries B's face on top.
Four thin flat spokes lie on A (bolt heads on top, printed on the bed).
Three spokes hang under B: a 4.5 mm plate carrying the bolt, with two 45-degree
gusset ribs that run down into the column, so every ceiling is a short bridge
or a 45-degree slope (no support needed when printed with A down).
"""
import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = POSE["bolts"]
ZA = POSE["mounting_faces"][0]["origin"][2]   # A face, normal +z
FB = POSE["mounting_faces"][1]["origin"]
ZB = FB[2]                                    # B face, normal -z
CX, CY = FB[0], FB[1]                         # column on B's centre

P = dict(tube_od=24.0, tube_id=18.0, a_t=4.0, a_w=10.0,
         b_t=4.5, rib_in=5.0, rib_t=3.0, tip=5.0)


def local(b):
    x, y, _ = b["position"]
    dx, dy = x - CX, y - CY
    return math.hypot(dx, dy), math.degrees(math.atan2(dy, dx))


def a_spoke(b):
    L, ang = local(b)
    s = extrude(SlotCenterToCenter(L, P["a_w"]), amount=P["a_t"])
    return Pos(CX, CY, ZA) * Rot(Z=ang) * Pos(L / 2, 0, 0) * s


def b_spoke(b):
    L, ang = local(b)
    zb = ZB - P["b_t"]
    xe = L + P["tip"]
    hw = P["rib_in"] + P["rib_t"]
    plate = Pos(xe / 2, 0, zb + P["b_t"] / 2) * Box(xe, 2 * hw, P["b_t"])
    # gusset rib profile in the XZ plane (x along the spoke), slope exactly 45 deg
    d = xe
    tri = Polygon((0, zb), (xe, zb), (0, zb - d), align=None)
    rib = extrude(tri, amount=P["rib_t"])      # extruded along +Y after rotation below (y in [0, rib_t])
    rib = Rot(X=90) * rib                      # sketch plane XY -> XZ, thickness along -Y
    r1 = Pos(0, -P["rib_in"] - P["rib_t"], 0) * rib  # y in [-in-t, -in]
    r2 = Pos(0, P["rib_in"], 0) * rib
    return Pos(CX, CY, 0) * Rot(Z=ang) * (plate + r1 + r2)


def build():
    part = Pos(CX, CY, ZA) * extrude(Circle(P["tube_od"] / 2), amount=ZB - ZA)
    for b in BOLTS:
        part += a_spoke(b) if b["id"][0] == "A" else b_spoke(b)
    part -= Pos(CX, CY, ZA) * extrude(Circle(P["tube_id"] / 2), amount=ZB - ZA)
    for b in BOLTS:
        d = 4.5 if b["size"] == "M4" else 5.5
        x, y, _ = b["position"]
        if b["id"][0] == "A":
            z0, t = ZA, P["a_t"]
        else:
            z0, t = ZB - P["b_t"], P["b_t"]
        part -= Pos(x, y, z0 - 0.5) * extrude(Circle(d / 2), amount=t + 1)
    return Part(part.wrapped)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
