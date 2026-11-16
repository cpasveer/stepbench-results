"""Connector for task pipe-L2-s1: flat square mount B -> round pipe A.

Concept (designed to print with the B seat face flat on the bed):
  * a square foot plate bolted to B (4x M4),
  * a V-block saddle that follows the pipe, with two flat lands square to the
    two bolt rows (4x M5),
  * one upright web between them, lying in the plane that contains the pipe
    axis and the normal of B, so the web is vertical on the printer.

Everything is modelled in the frame of B (X, Y in the mounting face, Z along
its outward normal = build direction) and moved to world coordinates at the end.
"""
import json
import math
import pathlib

import numpy as np
from build123d import Axis, Box, Plane, Polygon, Pos, Solid, export_step, extrude, fillet

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]

# ---- parameters (mm) -------------------------------------------------------
R_PIPE = TASK["fixed_parts"][0]["interface"]["params"]["od"] / 2
T_PLATE = 6.0          # foot plate thickness = M4 grip
PLATE = 36.0           # foot plate side (1 mm inside the 38 mm face)
R_CORNER = 3.0         # plate corner radius
GRIP_A = 6.0           # saddle thickness under the M5 heads
PAD_HALF_LEN = 25.0    # saddle half length along the pipe
PAD_SIDE = 7.0         # saddle width beyond each bolt row
D_M4, D_M5 = 4.5, 5.5  # clearance holes
WEB_Y0, WEB_Y1 = -7.5, 0.5   # web faces (between the M4 heads / clear of the M5 heads)
WEB_LEAN = 42.0        # overhang of the web's leaning edge, degrees from vertical
WEB_IN = 6.0           # how far the web runs into the saddle
WEB_END = 5.0          # web stops this far short of the saddle's upper end

# ---- frames ----------------------------------------------------------------
RA = np.array(POSE["placements"]["A"]["rotation"]); OA = np.array(POSE["placements"]["A"]["origin"])
RB = np.array(POSE["placements"]["B"]["rotation"]); OB = np.array(POSE["placements"]["B"]["origin"])


def to_b(p):            # world point -> B frame
    return RB.T @ (np.asarray(p) - OB)


def dir_b(v):           # world direction -> B frame
    return RB.T @ np.asarray(v)


C = to_b(OA)                    # a point on the pipe axis
D = dir_b(RA[:, 0])             # pipe axis
bolts_a = [b for b in BOLTS if b["id"].startswith("A")]
bolts_b = [b for b in BOLTS if b["id"].startswith("B")]
a1 = dir_b(bolts_a[0]["axis"])  # row A1/A2
a3 = dir_b(bolts_a[2]["axis"])  # row A3/A4
s = (a1 + a3) / np.linalg.norm(a1 + a3)      # middle of the saddle
t1 = np.cross(D, a1); t1 *= np.sign(np.dot(t1, a1 - a3))   # sideways, away from the middle
t3 = np.cross(D, a3); t3 *= np.sign(np.dot(t3, a3 - a1))
YB = np.array([0.0, 1.0, 0.0])
assert abs(D[1]) < 1e-9, "web plane assumption: pipe axis parallel to the XZ plane of B"


def solve2(n1, c1, n2, c2):
    """Point r (in the plane through C square to the pipe) with n1.r=c1, n2.r=c2."""
    A = np.array([n1, n2, D]); return np.linalg.solve(A, np.array([c1, c2, 0.0]))


def build():
    # ---- saddle: prism along the pipe ------------------------------------
    h = R_PIPE + GRIP_A
    wall = WEB_Y1 - C[1]                                 # plane y = WEB_Y1
    pts = [
        PAD_SIDE * t1 + 0.5 * R_PIPE * a1,               # inside the bore, A1 side
        PAD_SIDE * t1 + h * a1,                          # edge of land A1
        solve2(a1, h, YB, wall),                         # land A1 / web face
        solve2(a3, h, YB, wall),                         # web face / land A3
        PAD_SIDE * t3 + h * a3,                          # edge of land A3
        PAD_SIDE * t3 + 0.5 * R_PIPE * a3,               # inside the bore, A3 side
    ]
    ex, ey = t1, np.cross(D, t1)                         # sketch axes, normal = D
    sk_plane = Plane(origin=tuple(C - PAD_HALF_LEN * D), x_dir=tuple(ex), z_dir=tuple(D))
    prof = Polygon(*[(float(np.dot(p, ex)), float(np.dot(p, ey))) for p in pts], align=None)
    pad = extrude(sk_plane * prof, 2 * PAD_HALF_LEN, dir=tuple(D))
    pipe = Solid.make_cylinder(R_PIPE, 200, Plane(origin=tuple(C - 100 * D), z_dir=tuple(D)))
    pad = pad - pipe

    # ---- foot plate --------------------------------------------------------
    plate = Pos(0, 0, T_PLATE / 2) * Box(PLATE, PLATE, T_PLATE)
    plate = fillet(plate.edges().filter_by(Axis.Z), R_CORNER)

    # ---- web: vertical slab, profile in XZ ----------------------------------
    ym = 0.5 * (WEB_Y0 + WEB_Y1)
    # underside of the saddle (land A3 plane) in the web's mid plane: a3.(P-C) = h
    def land_z(x, depth=0.0):
        return C[2] + (a3[0] * (x - C[0]) + a3[1] * (ym - C[1]) - (h - depth)) / -a3[2]
    slope = (land_z(1.0) - land_z(0.0))
    half = PLATE / 2
    tl = math.tan(math.radians(WEB_LEAN))
    # leaning edge from the plate's top edge up into the saddle
    # z = T + (xl - x)/tl  meets  z = land_z(x, WEB_IN)
    x2 = (T_PLATE + (-half) / tl - land_z(0.0, WEB_IN)) / (slope + 1 / tl)
    p2 = (x2, land_z(x2, WEB_IN))
    # back edge from the other plate edge to a point short of the saddle's upper end
    x_end = (np.dot(D, C) + PAD_HALF_LEN - WEB_END - D[2] * land_z(0.0)) / (D[0] + D[2] * slope)
    # (x_end is on the land; move in by WEB_IN along the line plate-edge -> that point)
    e = np.array([x_end - half, land_z(x_end) - T_PLATE]); e /= np.linalg.norm(e)
    k = WEB_IN / abs(np.dot(e, np.array([-a3[0], -a3[2]]) / math.hypot(a3[0], a3[2])))
    p3 = (x_end + k * e[0], land_z(x_end) + k * e[1])
    web_pts = [(-half, T_PLATE / 2), (-half, T_PLATE), p2, p3, (half, T_PLATE), (half, T_PLATE / 2)]
    web_plane = Plane(origin=(0, WEB_Y1, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    web = extrude(web_plane * Polygon(*web_pts, align=None), WEB_Y1 - WEB_Y0, dir=(0, -1, 0))

    part = (plate + web + pad)

    # ---- holes ---------------------------------------------------------------
    for b in bolts_b:
        p, ax = to_b(b["position"]), dir_b(b["axis"])
        part -= Solid.make_cylinder(D_M4 / 2, T_PLATE + 2, Plane(origin=tuple(p - ax), z_dir=tuple(ax)))
    for b in bolts_a:
        p, ax = to_b(b["position"]), dir_b(b["axis"])
        part -= Solid.make_cylinder(D_M5 / 2, GRIP_A + 6, Plane(origin=tuple(p - 3 * ax), z_dir=tuple(ax)))
    part = part.clean()

    # ---- to world --------------------------------------------------------------
    world = Plane(origin=tuple(OB), x_dir=tuple(RB[:, 0]), z_dir=tuple(RB[:, 2]))
    return part.solids()[0].moved(world.location) if len(part.solids()) == 1 else part.moved(world.location)


if __name__ == "__main__":
    result = build()
    export_step(result, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
