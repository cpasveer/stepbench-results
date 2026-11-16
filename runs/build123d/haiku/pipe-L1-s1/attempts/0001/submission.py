"""Connecting bracket for pipe-L1-s1.

Pipe side: a curved saddle pad (8 mm thick, inner face on the 68 mm pipe)
carries the four M5 heads. A 12 x 8 mm rib rises from the pad centre, between
the heads, to the square mount. The 34 x 34 x 6 mm base plate sits on the
38 mm mounting face with the four M4 clearance holes on the 25 mm square.
Pad and plate are modelled in their fixed part's frame and placed with the
pose in task.json; the rib joins the two.
"""
import json
import math
import pathlib

from build123d import (Axis, Box, BuildSketch, Cylinder, Plane, Polygon,
                       Solid, Vector, export_step, extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
PL = TASK["poses"][0]["placements"]


def local_plane(placement):
    """Plane whose local axes are the fixed part's local axes, in world."""
    rot = placement["rotation"]
    x, z = (Vector(rot[0][j], rot[1][j], rot[2][j]) for j in (0, 2))
    return Plane(origin=Vector(*placement["origin"]), x_dir=x, z_dir=z)


PA = local_plane(PL["A"])  # pipe frame: X = pipe axis, bolts on the +Z side
PB = local_plane(PL["B"])  # mount frame: Z = face normal, face at z = 0

PIPE_R = 34.0
PAD_T = 8.0           # grip under the M5 heads
PAD_HALF_X = 26.0     # axial half length of the saddle
PAD_HALF_ANG = 50.0   # angular half width (deg) about the +Z bolt side
BOLT_ANG = 35.0       # bolt angle about the pipe, from +Z
HOLE_A = 5.5          # M5 clearance
PLATE = 34.0
PLATE_T = 6.0
PLATE_HOLE = 4.5      # M4 clearance
PITCH_B = 12.5        # half of the 25 mm square


def pad_local():
    """Saddle pad in pipe-local coordinates (pipe axis = X)."""
    length = 2 * PAD_HALF_X + 20
    outer = Solid.make_cylinder(PIPE_R + PAD_T, length,
                                plane=Plane(origin=(-length / 2, 0, 0), z_dir=(1, 0, 0)))
    inner = Solid.make_cylinder(PIPE_R, length + 2,
                                plane=Plane(origin=(-length / 2 - 1, 0, 0), z_dir=(1, 0, 0)))
    shell = outer - inner
    a = math.radians(PAD_HALF_ANG)
    r = 60.0
    pts = [(0, 0), (-r * math.sin(a), r * math.cos(a)), (0, r),
           (r * math.sin(a), r * math.cos(a))]
    with BuildSketch(Plane.YZ) as sk:
        Polygon(*pts)
    wedge = extrude(sk.sketch, amount=60, both=True)
    slab = Box(2 * PAD_HALF_X, 200, 200)
    pad = shell & wedge & slab
    for x in (-17.5, 17.5):
        for sgn in (-1, 1):
            phi = math.radians(BOLT_ANG) * sgn
            d = Vector(0, math.sin(phi), math.cos(phi))
            hole = Solid.make_cylinder(HOLE_A / 2, 30,
                                       plane=Plane(origin=Vector(x, 0, 0) + d * 28, z_dir=d))
            pad = pad - hole
    return pad


def plate_local():
    """Base plate in mount-local coordinates (face at z = 0)."""
    plate = Box(PLATE, PLATE, PLATE_T).moved(Plane.XY.location)
    plate = plate.moved(
        __import__("build123d").Location((0, 0, PLATE_T / 2)))
    plate = plate.fillet(3.0, plate.edges().filter_by(Axis.Z))
    for sx in (-1, 1):
        for sy in (-1, 1):
            hole = Solid.make_cylinder(PLATE_HOLE / 2, 30,
                                       plane=Plane(origin=(sx * PITCH_B, sy * PITCH_B, -15)))
            plate = plate - hole
    return plate


def build():
    pad = pad_local().moved(PA.location)
    plate = plate_local().moved(PB.location)

    # Rib: from the plate top (between the bolts) to the pad top centre.
    pad_top = PA.from_local_coords((0, 0, PIPE_R + PAD_T))
    plate_pt = PB.from_local_coords((-8.0, 0.0, PLATE_T))
    w = (pad_top - plate_pt).normalized()
    start = plate_pt - w * 3.0   # 3 mm into the plate
    end = pad_top + w * 3.0      # 3 mm into the pad
    span = (end - start).length
    mid = (start + end) * 0.5
    xdir = PA.x_dir - w * PA.x_dir.dot(w)
    xdir = xdir.normalized()
    rib = Box(12.0, 8.0, span).moved(
        Plane(origin=mid, x_dir=xdir, z_dir=w).location)

    body = pad.fuse(plate).fuse(rib)
    return body.clean()


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
