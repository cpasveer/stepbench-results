"""Hub-and-spoke connector: a hollow central column, four thin spokes on A, three thicker spokes on B."""
import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = POSE["bolts"]
ZA = POSE["mounting_faces"][0]["origin"][2]   # A face, normal +z
ZB = POSE["mounting_faces"][1]["origin"][2]   # B face, normal -z

CX, CY = -125.0, 131.0
P = dict(tube_od=22.0, tube_id=15.0, a_t=4.0, a_w=10.0, b_t=8.0, b_w=12.0)




def spoke(b, z0, t, w):
    x, y, _ = b["position"]
    dx, dy = x - CX, y - CY
    L = (dx * dx + dy * dy) ** 0.5
    ang = math.degrees(math.atan2(dy, dx))
    s = extrude(SlotCenterToCenter(L, w), amount=t)  # slot along local x, centred at origin
    s = Pos(L / 2, 0, 0) * s
    return Pos(CX, CY, z0) * Rot(Z=ang) * s


def build():
    a = [b for b in BOLTS if b["id"].startswith("A")]
    bb = [b for b in BOLTS if b["id"].startswith("B")]
    part = Pos(CX, CY, ZA) * extrude(Circle(P["tube_od"] / 2), amount=ZB - ZA)
    for b in a:
        part += spoke(b, ZA, P["a_t"], P["a_w"])
    for b in bb:
        part += spoke(b, ZB - P["b_t"], P["b_t"], P["b_w"])
    part -= Pos(CX, CY, ZA) * extrude(Circle(P["tube_id"] / 2), amount=ZB - ZA)
    for b in BOLTS:
        d = 4.5 if b["size"] == "M4" else 5.5
        x, y, z = b["position"]
        part -= Pos(x, y, (ZA + ZB) / 2) * extrude(Circle(d / 2), amount=ZB - ZA, both=True)
    return Part(part.wrapped)


if __name__ == "__main__":
    import math
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
