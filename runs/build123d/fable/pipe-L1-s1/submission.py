"""Connector for task pipe-L1-s1: pipe saddle + oblique strut + square flange.

The part is one solid made of three members:
  * a saddle that bears on the pipe (bore = pipe radius) with two flat lands,
    square to the two rows of M5 bolts, so the heads seat on flat faces;
  * a 36 x 36 mm flange on the flat mount B (1 mm inside its outline);
  * a rectangular strut running straight from the saddle crown to the flange,
    threaded between the flange bolts so every head and hex key stays free.

    python submission.py        # writes output.step
"""
import json
import pathlib

import numpy as np
from build123d import (Align, Axis, Box, BuildLine, BuildSketch, Cylinder, Location,
                       Part, Plane, Polyline, Pos, Rectangle, Vector, export_step,
                       extrude, fillet, make_face)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
PLACE = TASK["poses"][0]["placements"]
RA, OA = np.array(PLACE["A"]["rotation"]), np.array(PLACE["A"]["origin"])
RB, OB = np.array(PLACE["B"]["rotation"]), np.array(PLACE["B"]["origin"])
IFA = TASK["fixed_parts"][0]["interface"]
IFB = TASK["fixed_parts"][1]["interface"]

# ---- design parameters (mm) -------------------------------------------------
R_PIPE = IFA["params"]["od"] / 2      # 34
A_HOLE_D = 5.5                         # M5 clearance
B_HOLE_D = 4.5                         # M4 clearance
A_GRIP = 7.0                           # saddle thickness under the M5 heads
SADDLE_TOP = 6.0                       # saddle thickness at the crown
SADDLE_HALF_LEN = 27.5                 # along the pipe (bolts at +-17.5)
SADDLE_END_ANGLE = 57.0                # deg from crown to the saddle's lower edge
FLANGE = 36.0                          # square flange on B (face is 38)
FLANGE_T = 6.0
FLANGE_CORNER_R = 4.0
STRUT_W = 12.0                         # across (between the B bolt pairs)
STRUT_L = 22.0                         # along the pipe direction
STRUT_AT_B = (1.5, 2.0)                # strut centre on B's face, B-local
STRUT_AT_A = (0.0, -5.0, R_PIPE + SADDLE_TOP)   # strut foot on the saddle crown, pipe-local


def frame(R, O) -> Location:
    """Location of a fixed part's local frame in the world."""
    return Plane(origin=tuple(O), x_dir=tuple(R[:, 0]), z_dir=tuple(R[:, 2])).location


def build() -> Part:
    loc_a, loc_b = frame(RA, OA), frame(RB, OB)

    # --- saddle, in the pipe's frame (x = pipe axis, bolts about +z) ---------
    row_ang = np.degrees(np.arctan2(abs(IFA["hole_frames_local"][0]["axis"][1]),
                                    IFA["hole_frames_local"][0]["axis"][2]))  # 35
    r_land = R_PIPE + A_GRIP
    z_top = R_PIPE + SADDLE_TOP
    s, c = np.sin(np.radians(row_ang)), np.cos(np.radians(row_ang))
    y_ridge = (r_land - z_top * c) / s                 # crown flat meets land
    se, ce = np.sin(np.radians(SADDLE_END_ANGLE)), np.cos(np.radians(SADDLE_END_ANGLE))
    r_end = r_land / np.cos(np.radians(SADDLE_END_ANGLE - row_ang))
    r_in = R_PIPE - 6.0                                # inside the bore, cut away later
    pts = [(-r_in * se, r_in * ce), (-r_end * se, r_end * ce), (-y_ridge, z_top),
           (y_ridge, z_top), (r_end * se, r_end * ce), (r_in * se, r_in * ce),
           (0.0, 4.0)]
    with BuildSketch(Plane.YZ) as sk:
        with BuildLine():
            Polyline(*pts, close=True)
        make_face()
    saddle = extrude(sk.sketch, amount=SADDLE_HALF_LEN, both=True)
    saddle = loc_a * saddle

    # --- flange, in B's frame (z = outward normal) ---------------------------
    flange = Box(FLANGE, FLANGE, FLANGE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    flange = fillet(flange.edges().filter_by(Axis.Z), FLANGE_CORNER_R)
    flange = loc_b * flange

    # --- strut: straight rectangular bar from the crown to the flange --------
    p_b = OB + RB @ np.array([STRUT_AT_B[0], STRUT_AT_B[1], 0.0])
    p_a = OA + RA @ np.array(STRUT_AT_A)
    u = (p_b - p_a) / np.linalg.norm(p_b - p_a)
    v1 = RB[:, 1] - (RB[:, 1] @ u) * u                 # width: square to the pipe axis
    v1 /= np.linalg.norm(v1)
    length = float(np.linalg.norm(p_b - p_a))
    pl = Plane(origin=tuple(p_a - 14.0 * u), x_dir=tuple(v1), z_dir=tuple(u))
    strut = extrude(pl * Rectangle(STRUT_W, STRUT_L), amount=length + 14.0 + 30.0)
    # end it on B's mounting plane
    strut -= loc_b * Box(400, 400, 200, align=(Align.CENTER, Align.CENTER, Align.MAX))

    part = saddle + strut + flange

    # --- pipe bore and bolt holes -------------------------------------------
    part -= loc_a * Cylinder(R_PIPE, 400, rotation=(0, 90, 0))
    for hf in IFA["hole_frames_local"]:
        p, a = np.array(hf["point"]), np.array(hf["axis"])
        hp = Plane(origin=tuple(OA + RA @ (p - 3 * a)), z_dir=tuple(RA @ a))
        part -= hp * Cylinder(A_HOLE_D / 2, A_GRIP + 6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for hx, hy in IFB["holes_local"]:
        part -= loc_b * Pos(hx, hy, -3) * Cylinder(
            B_HOLE_D / 2, FLANGE_T + 6, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.clean()
    return part


if __name__ == "__main__":
    result = build()
    assert len(result.solids()) == 1 and result.is_valid
    export_step(result, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(result.volume, 1))
