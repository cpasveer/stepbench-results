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
         b_t=4.5, w=12.0, k=1.1, pocket_r=4.75)


def local(b):
    x, y, _ = b["position"]
    dx, dy = x - CX, y - CY
    return math.hypot(dx, dy), math.degrees(math.atan2(dy, dx))


def a_spoke(b):
    L, ang = local(b)
    s = extrude(SlotCenterToCenter(L, P["a_w"]), amount=P["a_t"])
    return Pos(CX, CY, ZA) * Rot(Z=ang) * Pos(L / 2, 0, 0) * s


def b_spoke(b):
    """Solid 45-ish degree wedge under B's face: flat plate on top, underside ramps
    down to the column; the bolt head sits in a round pocket (10 mm bridge)."""
    L, ang = local(b)
    zb = ZB - P["b_t"]
    xe = L + P["w"] / 2                       # tip of the rounded planform
    plan = extrude(SlotCenterToCenter(L, P["w"]), amount=ZB - (zb - P["k"] * xe) + 1)
    plan = Pos(L / 2, 0, zb - P["k"] * xe - 1) * plan
    tri = Polygon((0, ZB), (xe, ZB), (xe, zb), (0, zb - P["k"] * xe), align=None)
    prof = extrude(tri, amount=P["w"] + 2)
    prof = Pos(0, -(P["w"] + 2) / 2, 0) * (Rot(X=90) * prof)   # rot maps z->-y: fix below
    return Pos(CX, CY, 0) * Rot(Z=ang) * (plan & prof)


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
        if b["id"][0] == "B":
            zlo = z0 - P["k"] * (local(b)[0] + P["w"] / 2) - 2   # below the wedge only
            part -= Pos(x, y, zlo) * extrude(Circle(P["pocket_r"]), amount=z0 - zlo)
    return Part(part.wrapped)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
