"""Connector for facing-L3-s1: a three-web truss bracket, printed on its side.

Design frame (u, h, z): h is the print-up direction (horizontal in the world,
along one side of A's bolt square), u is across, z is the world vertical.
Every wall is vertical in print: the two flanges, the two upright webs and the
diagonal web that triangulates them.  Bolt tabs stick out sideways from the
webs so every bolt has free tool and key access on the outside.
"""
import json
import math
import pathlib

from build123d import (Align, Axis, Box, Cylinder, Location, Plane, Polygon, Pos,
                       Part, Solid, Vector, export_step, extrude, revolve)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]
PA = POSE["placements"]["A"]
PB = POSE["placements"]["B"]

ZA = PA["origin"][2]            # A mounting face
ZB = PB["origin"][2]            # B mounting face
TA = 3.0                        # flange on A
TB = 3.8                        # flange on B
TW = 1.2                        # webs
BED = 33.7                      # h = BED - local x of A
EX = Vector(PA["rotation"][0][0], PA["rotation"][1][0], 0)   # A local x in world
EY = Vector(PA["rotation"][0][1], PA["rotation"][1][1], 0)   # A local y in world
AC = Vector(*PA["origin"])
ORG = AC + EX * BED             # world position of (u=0, h=0)
ANG = math.degrees(math.atan2(EY.Y, EY.X))
SLOPE = math.tan(math.radians(48))   # tab undersides: 42 deg overhang in print


def to_design(p):
    d = Vector(p[0], p[1], 0) - Vector(ORG.X, ORG.Y, 0)
    return d.dot(EY), -d.dot(EX)


def slab(pts, z0, z1):
    """Prism with outline pts in (u, h), from z0 to z1."""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1]))
    if area < 0:
        pts = pts[::-1]
    return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), z1 - z0)


def rect(u0, u1, h0, h1):
    return [(u0, h0), (u1, h0), (u1, h1), (u0, h1)]


def build() -> Part:
    U1, U2 = -22.8, 15.2        # outer faces of the upright webs
    HT = 67.7                   # top of the flanges and upright webs in print
    HD = 45.0                   # top of the diagonal web
    RIM = 5.6                   # flange left beside every web foot
    SILL = 6.0                  # flange tie left along the bed edge
    zA1, zB0 = ZA + TA, ZB - TB
    bu, bh = to_design(PB["origin"])
    ub, ut = 4.0, -15.0         # diagonal: foot on A, head on B
    hw = 0.5 * TW / math.sin(math.atan2(zB0 - zA1, abs(ut - ub)))

    parts = []
    # flanges: a strip under every web foot, tied along the bed edge, open above
    fa = slab(rect(U1, U2, 0, HT), ZA, zA1) - slab(
        rect(U1 + TW + RIM, ub - hw - RIM, SILL, HT + 1), ZA - 1, zA1 + 1)
    fb = slab(rect(U1, U2, 0, HT), zB0, ZB) - slab(
        rect(ut + hw + RIM, U2 - TW - RIM, SILL, HT + 1), zB0 - 1, ZB + 1)
    parts += [fa, fb]
    # upright webs and the diagonal
    parts.append(slab(rect(U1, U1 + TW, 0, HT), zA1, zB0))
    parts.append(slab(rect(U2 - TW, U2, 0, HT), zA1, zB0))
    diag = extrude(Plane.XZ * Polygon((ub - hw, zA1), (ub + hw, zA1), (ut + hw, zB0), (ut - hw, zB0), align=None), -HD)
    parts.append(diag)

    # tabs on A (bolts at u = +-30)
    EA = 34.5
    parts.append(slab(rect(-EA, U1, 0, 8.2), ZA, zA1))
    parts.append(slab(rect(U2, EA, 0, 8.2), ZA, zA1))
    h0 = 59.2
    parts.append(slab([(U1, h0 - (EA + U1) * SLOPE), (-EA, h0), (-EA, HT), (U1, HT)], ZA, zA1))
    parts.append(slab([(U2, h0 - (EA - U2) * SLOPE), (EA, h0), (EA, HT), (U2, HT)], ZA, zA1))
    # tabs on B
    parts.append(slab(rect(U2, 30.4, 0, 14.7), zB0, ZB))                       # B3
    e2 = 24.95
    parts.append(slab([(U2, 66.1 - (e2 - U2) * SLOPE), (e2, 66.1), (e2, 74.9), (U2, 74.9)], zB0, ZB))   # B2
    e1 = -33.5
    parts.append(slab([(U1, 31.3 - (U1 - e1) * SLOPE), (e1, 31.3), (e1, 40.3), (U1, 40.3)], zB0, ZB))   # B1

    body = parts[0]
    for p in parts[1:]:
        body = body + p

    # relief cone: outside B's seat circle the flange falls away from B
    r0, r1 = 40.9, 75.0
    drop = (r1 - r0) * math.tan(math.radians(12))
    cone = revolve(Plane.XZ * Polygon((r0, ZB), (r1, ZB), (r1, ZB - drop), align=None), Axis.Z)
    body = body - Pos(bu, bh, 0) * cone

    # bolt holes
    for b in BOLTS:
        u, h = to_design(b["position"])
        d = 4.5 if b["size"] == "M4" else 5.5
        zc = ZA + 2 if b["size"] == "M4" else ZB - 2
        body = body - Pos(u, h, zc) * Cylinder(d / 2, 10)

    body = body.clean()
    return Location((ORG.X, ORG.Y, 0), (0, 0, ANG)) * body


if __name__ == "__main__":
    part = build()
    s = part.solids()
    print("solids", len(s), "valid", part.is_valid, "volume", part.volume, "faces", len(part.faces()))
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
