"""Connector: central column; flat foot-arms to A on the bottom, sloped
(self-supporting) arms to B on top with counterbored head seats."""
import json, math, pathlib
from build123d import (Part, Plane, Polygon, Cylinder, Box, Pos, Rot, Align, extrude, export_step)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
B = TASK["poses"][0]["resolved"]["bolts"]
PX, PY = -126.0, 127.5           # column axis = centre of A
ZA, ZB = -94.0, -25.5
T = 6.0                           # plate thickness under heads
POCKET = 2.0                      # counterbore depth on B side
HW = 5.0                          # arm half width
COLR = 10.0
TAN = math.tan(math.radians(58))
MIN = Align.MIN
CEN = Align.CENTER


def arm(bolt, lower):
    bx, by = bolt["position"][0] - PX, bolt["position"][1] - PY
    D = math.hypot(bx, by)
    ang = math.degrees(math.atan2(by, bx))
    e = D + HW
    if lower:
        z0, z1 = ZA, ZA + T
        pts = [(0, z0), (e, z0), (e, z1), (0, z1)]
    else:
        z0, z1 = ZB, ZB - T - POCKET
        x0 = D - 8
        dd = x0 - (COLR - 2)
        pts = [(0, z0), (e, z0), (e, z1), (x0, z1), (COLR - 2, z1 - dd * TAN), (0, z1 - dd * TAN)]
    prof = Plane.XZ * Polygon(*pts, align=None)
    body = extrude(prof, HW, both=True)
    h = ZB - ZA + 2
    plan = Pos(D, 0, ZA - 1) * Cylinder(HW, h, align=(CEN, CEN, MIN))
    plan = plan + Pos(D / 2, 0, ZA - 1) * Box(D, 2 * HW, h, align=(CEN, CEN, MIN))
    return Rot(0, 0, ang) * (body & plan)


def build() -> Part:
    p = Cylinder(COLR, ZB - ZA, align=(CEN, CEN, MIN))
    p = Pos(0, 0, ZA) * p
    for b in B:
        p = p + arm(b, b["id"].startswith("A"))
    p = Pos(PX, PY, 0) * p
    for b in B:
        d = sum(b["clearance_diameter_range"]) / 2
        x, y, z = b["position"]
        p = p - Pos(x, y, ZA - 1) * Cylinder(d / 2, ZB - ZA + 2, align=(CEN, CEN, MIN))
        if b["id"].startswith("B"):
            p = p - Pos(x, y, ZB - T - POCKET - 0.001) * Cylinder(5.1, POCKET + 0.001 - T + T, align=(CEN, CEN, MIN)) if False else p
            p = p - Pos(x, y, ZB - T - POCKET - 1) * Cylinder(5.1, POCKET + 1, align=(CEN, CEN, MIN))
        else:
            p = p - Pos(x, y, ZA + T) * Cylinder(5.0, ZB - ZA - T + 1, align=(CEN, CEN, MIN))
    return Part(p.clean())


if __name__ == "__main__":
    s = build()
    print(s.volume, s.is_valid, len(s.solids()))
    export_step(s, str(HERE / "output.step"))
