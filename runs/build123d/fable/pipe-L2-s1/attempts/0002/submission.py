"""Connector for task pipe-L2-s1: flat square mount B -> round pipe A.

Concept (designed to print along the pipe axis, standing on one end of the
saddle, with no support):
  * a V-block saddle that follows the pipe, with two flat lands square to the
    two bolt rows (4x M5) -- a plain prism along the pipe axis,
  * a square foot plate bolted to B (4x M4); on the printer it is a shelf
    sloping at 40 degrees, carried by a lip that runs from its lower edge
    down to the bed,
  * one web between them, lying in the plane that contains the pipe axis and
    the normal of B, so it is an upright wall on the printer.

Everything is modelled in the frame of B (X, Y in the mounting face, Z along
its outward normal) and moved to world coordinates at the end.
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
WEB_IN = 6.0           # how far the web runs into the saddle
WEB_END = 5.0          # the web's top edge meets the saddle this far from its end

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

    # ---- foot plate with its lip (profile in XZ, extruded across Y) ----------
    half = PLATE / 2
    bed = float(np.dot(D, C)) + PAD_HALF_LEN             # bed plane: D.P = bed
    dx, dz = float(D[0]), float(D[2])

    def to_bed(x, z):                                    # run along the pipe axis to the bed
        t = bed - (dx * x + dz * z)
        return (x + t * dx, z + t * dz)

    side = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))   # sketch (x, z)
    foot_pts = [(-half, 0), (half, 0), to_bed(half, 0), to_bed(half, T_PLATE),
                (half, T_PLATE), (-half, T_PLATE)]
    plate = extrude(side * Polygon(*foot_pts, align=None), half, dir=(0, 1, 0), both=True)
    top_corners = plate.edges().filter_by(Axis.Z).filter_by(lambda e: e.center().X < -half + 0.01)
    plate = fillet(top_corners, R_CORNER)

    # ---- web: slab between y = WEB_Y0 and WEB_Y1, profile in XZ ----------------
    ym = 0.5 * (WEB_Y0 + WEB_Y1)

    def land(lx, depth=0.0):
        """Point of the saddle's underside (land A3), `depth` inside it, at pipe station lx."""
        A = np.array([[a3[0], a3[2]], [dx, dz]])
        rhs = np.array([h - depth + float(np.dot(a3, C)) - a3[1] * ym, lx + float(np.dot(D, C))])
        x, z = np.linalg.solve(A, rhs)
        return np.array([x, z])

    p1 = np.array([-half, T_PLATE])
    e = land(-PAD_HALF_LEN + WEB_END) - p1; e /= np.linalg.norm(e)
    # carry the top edge on to WEB_IN inside the saddle
    n_in = np.array([-a3[0], -a3[2]]) / math.hypot(a3[0], a3[2])
    p2 = land(-PAD_HALF_LEN + WEB_END) + e * (WEB_IN / float(np.dot(e, n_in)))
    p3 = land(PAD_HALF_LEN, WEB_IN)                      # on the bed, inside the saddle
    p4 = to_bed(half, T_PLATE / 2)                       # on the bed, inside the lip
    web_pts = [tuple(p1), tuple(p2), tuple(p3), p4, (half, T_PLATE / 2), (-half, T_PLATE / 2)]
    web = extrude(Plane(origin=(0, WEB_Y1, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
                  * Polygon(*web_pts, align=None), WEB_Y1 - WEB_Y0, dir=(0, -1, 0))

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
