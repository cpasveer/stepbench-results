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
HW = 5.0
HWU = 7.0
COLR = 10.0
TAN = math.tan(math.radians(58))
MIN = Align.MIN
CEN = Align.CENTER


def arm(bolt, lower):
    bx, by = bolt["position"][0] - PX, bolt["position"][1] - PY
    D = math.hypot(bx, by)
    ang = math.degrees(math.atan2(by, bx))
    hw = HW if lower else HWU
    e = D + hw
    if lower:
        z0, z1 = ZA, ZA + T
        pts = [(0, z0), (e, z0), (e, z1), (0, z1)]
    else:
        z0, z1 = ZB, ZB - T - POCKET
        x0 = D - 8
        dd = x0 - (COLR - 2)
        pts = [(0, z0), (e, z0), (e, z1), (x0, z1), (COLR - 2, z1 - dd * TAN), (0, z1 - dd * TAN)]
    prof = Plane.XZ * Polygon(*pts, align=None)
    body = extrude(prof, hw, both=True)
    h = ZB - ZA + 2
    plan = Pos(D, 0, ZA - 1) * Cylinder(hw, h, align=(CEN, CEN, MIN))
    plan = plan + Pos(D / 2, 0, ZA - 1) * Box(D, 2 * hw, h, align=(CEN, CEN, MIN))
    return Rot(0, 0, ang) * (body & plan)


def build() -> Part:
    p = Cylinder(COLR, ZB - ZA, align=(CEN, CEN, MIN))
    p = Pos(0, 0, ZA) * p
    for b in B:
        p = p + arm(b, b["id"].startswith("A"))
    p = Pos(PX, PY, 0) * p
    bc = TASK["poses"][0]["placements"]["B"]["origin"]
    clip = Pos(bc[0], bc[1], -50) * (Cylinder(90, ZB + 1 + 50, align=(CEN, CEN, MIN)) - Cylinder(41.0, ZB + 1 + 50, align=(CEN, CEN, MIN)))
    p = p - clip
    for b in B:
        d = sum(b["clearance_diameter_range"]) / 2
        x, y, z = b["position"]
        p = p - Pos(x, y, ZA - 1) * Cylinder(d / 2, ZB - ZA + 2, align=(CEN, CEN, MIN))
        if b["id"].startswith("B"):
            p = p - Pos(x, y, ZA - 1) * Cylinder(4.8, ZB - T - ZA + 1, align=(CEN, CEN, MIN))
        else:
            p = p - Pos(x, y, ZA + T) * Cylinder(5.0, ZB - ZA - T + 1, align=(CEN, CEN, MIN))
    return Part(p.clean())


if __name__ == "__main__":
    s = build()
    print(s.volume, s.is_valid, len(s.solids()))
    export_step(s, str(HERE / "output.step"))
