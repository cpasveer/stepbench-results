"""Connector: two seat plates joined by a central beam, clear of all key sweeps."""
import json, pathlib
from build123d import (Box, Cylinder, Plane, Pos, Vector, export_step, Part)

HERE = pathlib.Path(__file__).parent
POSE = json.loads((HERE / "task.json").read_text())["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
T = 8.0  # plate thickness (grip)


def face_plane(f):
    n = Vector(*f["normal"])
    t = Vector(0, 0, 1).cross(n).normalized()
    return Plane(origin=Vector(*f["origin"]), x_dir=t, z_dir=n)


def build() -> Part:
    pa, pb = face_plane(FACES["A"]), face_plane(FACES["B"])
    plate_a = pa.location * (Pos(0, 0, T / 2) * Cylinder(36.0, T))
    # B face centre is the origin given; outline 46 x 96, keep 1.2 mm inside
    plate_b = pb.location * (Pos(0, 0, T / 2) * Box(43.6, 93.6, T))
    a0 = Vector(*FACES["A"]["origin"]) + Vector(*FACES["A"]["normal"]) * 4
    b0 = Vector(*FACES["B"]["origin"]) + Vector(*FACES["B"]["normal"]) * 4
    a0 = Vector(a0.X, a0.Y, 0); b0 = Vector(b0.X, b0.Y, 0)
    d = b0 - a0
    L = d.length
    beam_pl = Plane(origin=(a0 + b0) / 2, x_dir=d.normalized(), z_dir=Vector(0, 0, 1))
    beam = beam_pl.location * Box(L, 18, 24)
    part = plate_a + plate_b + beam
    for b in BOLTS:
        dia = 5.5 if b["size"] == "M5" else 6.7
        ax = Vector(*b["axis"])
        pl = Plane(origin=Vector(*b["position"]) - ax * 1, z_dir=ax)
        part = part - pl.location * (Pos(0, 0, 6) * Cylinder(dia / 2, 12))
    return Part(part.wrapped) if not isinstance(part, Part) else part


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
