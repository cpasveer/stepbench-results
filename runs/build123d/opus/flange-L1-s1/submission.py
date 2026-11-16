"""Adapter bracket joining round flange A (6x M5, PCD 60) to rectangular pad B
(4x M6, 30 x 80 grid).

Concept: a round seat plate on A and a rectangular seat plate on B, joined by
one lofted strut (circle on A to ellipse on B, one smooth face) that runs from the middle of the A plate to the middle of the
B plate. The strut stays inside both bolt patterns, so every cap screw head is
free and the hex keys can be swung outward, clear of the part.
"""
import json
import pathlib

from build123d import (
    Axis, Circle, Ellipse, Cylinder, Edge, Location, Plane, Pos, RectangleRounded,
    Solid, Vector, export_step, loft, extrude, fillet, Part,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
FACES = {f["part"]: f for f in POSE["resolved"]["mounting_faces"]}
BOLTS = POSE["resolved"]["bolts"]
PLACE = POSE["placements"]

# --- design parameters -------------------------------------------------------
T_A = 8.0            # seat plate thickness on A (M5 grip)
T_B = 9.0            # seat plate thickness on B (M6 grip)
R_PLATE_A = 35.25    # A face is 74.5 dia -> 37.25 radius; stay 2 mm inside
B_W, B_H, B_R = 42.0, 92.0, 6.0   # B face is 46 x 96; stay 2 mm inside
EDGE_FILLET = 1.5    # top outer edge of each plate
HOLE = {"M5": 5.5, "M6": 6.6}     # ISO 273 medium clearance
STRUT_A_R = 15.0     # strut section on A plate (circle)
STRUT_B = (11.5, 24.0)           # strut section on B plate (ellipse semi-axes)


def frame(name: str) -> Plane:
    rot = PLACE[name]["rotation"]
    x_dir = (rot[0][0], rot[1][0], rot[2][0])
    z_dir = tuple(FACES[name]["normal"])
    return Plane(origin=tuple(FACES[name]["origin"]), x_dir=x_dir, z_dir=z_dir)


def build() -> Part:
    pa, pb = frame("A"), frame("B")

    plate_a = pa * extrude(Circle(R_PLATE_A), T_A)
    plate_a = fillet(plate_a.edges().sort_by(Axis((0, 0, 0), tuple(pa.z_dir)))[-1:], EDGE_FILLET)

    plate_b = pb * extrude(RectangleRounded(B_W, B_H, B_R), T_B)
    top_b = plate_b.faces().sort_by(Axis((0, 0, 0), tuple(pb.z_dir)))[-1]
    plate_b = fillet(top_b.outer_wire().edges(), EDGE_FILLET)

    # strut: loft from the top face of plate A to the top face of plate B,
    # sections centred on each plate.
    sec_a = pa.offset(T_A) * Circle(STRUT_A_R)
    sec_b = pb.offset(T_B) * Ellipse(*STRUT_B)
    strut = loft([sec_a, sec_b])

    part = plate_a + plate_b + strut

    # clearance holes, drilled through each plate along the bolt axis
    for b in BOLTS:
        t = T_A if b["id"].startswith("A") else T_B
        d = HOLE[b["size"]]
        ax = Vector(*b["axis"])
        p0 = Vector(*b["position"]) - ax * 1.0
        hole = Solid.make_cylinder(d / 2, t + 2.0, Plane(origin=p0, z_dir=ax))
        part = part - hole

    return part.clean()


if __name__ == "__main__":
    p = build()
    print("solids:", len(p.solids()), "valid:", p.is_valid, "volume:", round(p.volume))
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
