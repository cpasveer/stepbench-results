"""Connector: central column, three-/four-arm star with sloped (self-supporting) webs."""
import json, math, pathlib
from build123d import (Part, Plane, Polygon, Circle, Cylinder, Pos, Rot, Align,
                       extrude, make_face, Locations, Axis, export_step, Vector, Sketch, BuildSketch, BuildPart, Mode)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
B = TASK["poses"][0]["resolved"]["bolts"]
PX, PY = -126.0, 127.5           # column axis = centre of A
ZA, ZB = -94.0, -25.5
T = 6.0                           # plate thickness at bolts
HW = 6.5                          # half width of arm
COLR = 10.0
TAN = math.tan(math.radians(58))


def arm(bolt, lower):
    bx, by = bolt["position"][0] - PX, bolt["position"][1] - PY
    D = math.hypot(bx, by)
    ang = math.degrees(math.atan2(by, bx))
    e = D + HW
    if lower:
        z0, z1 = ZA, ZA + T
        pts = [(0, z0), (e, z0), (e, z1), (D - 8, z1)]
        dd = D - 8 - COLR + 2
        pts += [(COLR - 2, z1 + dd * TAN), (0, z1 + dd * TAN)]
    else:
        z0, z1 = ZB, ZB - T
        pts = [(0, z0), (e, z0), (e, z1), (D - 8, z1)]
        dd = D - 8 - COLR + 2
        pts += [(COLR - 2, z1 - dd * TAN), (0, z1 - dd * TAN)]
    # profile in XZ plane (x = d, z), extrude along y symmetric
    prof = Plane.XZ * Polygon(*[(x, z) for x, z in pts], align=None)
    body = extrude(prof, HW, both=True)
    zmin, zmax = ZA - 1, ZB + 1
    stad = Pos(0, 0, zmin) * extrude(Circle(HW), zmax - zmin) 
    plan = Pos(D, 0, zmin) * Cylinder(HW, zmax - zmin, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body = body & (plan + Pos(D / 2, 0, zmin) * __import__("build123d").Box(D, 2 * HW, zmax - zmin, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    return Rot(0, 0, ang) * body


def build() -> Part:
    p = Pos(PX, PY, ZA) * Cylinder(COLR, ZB - ZA, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = Pos(-PX, -PY, 0) * p  # work around the origin
    for b in B:
        lower = b["id"].startswith("A")
        p = p + arm(b, lower)
    p = Pos(PX, PY, 0) * p
    for b in B:
        d = sum(b["clearance_diameter_range"]) / 2
        x, y, z = b["position"]
        p = p - Pos(x, y, (ZA + ZB) / 2) * Cylinder(d / 2, ZB - ZA + 2)
    return Part(p.clean())


if __name__ == "__main__":
    s = build()
    print(s.volume, s.is_valid, len(s.solids()))
    export_step(s, str(HERE / "output.step"))
