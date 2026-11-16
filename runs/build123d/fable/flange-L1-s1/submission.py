"""Connector bracket for task flange-L1-s1 (build123d).

Two bolting plates (a round one on flange A, a rectangular one on block B)
joined by one straight box beam that passes between the screw rows of both
patterns, so every screw head stays free for its hex key.  Both mounting
faces are upright, so the beam is a plain prism along z, tapered in plan.

    python submission.py        # writes output.step
"""
import json
import pathlib

from build123d import (Axis, Cylinder, Face, Plane, Pos, Rectangle,
                       RectangleRounded, Solid, Vector, Wire, Circle,
                       chamfer, export_step, extrude, fillet)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]

# ---- design numbers -------------------------------------------------------
T_PLATE = 10.0       # plate thickness = grip of every screw
SETBACK = 1.0        # plate edge stays this far inside the mounting face
A_LAT = (-20.0, 20.0)   # beam footprint on plate A, across the face (horizontal)
B_LAT = (-12.0, 19.0)   # beam footprint on plate B, along its local x
BEAM_Z = (-13.0, 14.0)  # beam height range (world z), clear of every screw row
R_BEAM = 4.0         # rounding of the beam's long edges
R_ROOT = 1.5         # fillet where the beam runs into a plate
C_RIM = 1.0          # chamfer on the plates' outer front edges
HOLE = {"M5": 5.5, "M6": 6.6}   # ISO 273 medium clearance holes
Z = Vector(0, 0, 1)


def frame(name):
    pl = POSE["placements"][name]
    rot = pl["rotation"]
    col = lambda i: Vector(rot[0][i], rot[1][i], rot[2][i])
    return Vector(*pl["origin"]), col(0), col(1), col(2)


oA, xA, yA, nA = frame("A")
oB, xB, yB, nB = frame("B")
uA = Z.cross(nA).normalized()      # horizontal direction in face A
uB = Z.cross(nB).normalized()      # horizontal direction in face B
faceA = next(f for f in TASK["fixed_parts"] if f["name"] == "A")["interface"]["face_size"]
faceB = next(f for f in TASK["fixed_parts"] if f["name"] == "B")["interface"]["face_size"]


def build():
    # --- plates -----------------------------------------------------------
    rA = faceA[0] / 2 - SETBACK
    plate_a = extrude(Plane(origin=oA, x_dir=uA, z_dir=nA) * Circle(rA), T_PLATE)
    # corners of plate B are concentric with the screws
    pitch_x = 2 * abs(next(f for f in TASK["fixed_parts"] if f["name"] == "B")
                      ["interface"]["holes_local"][0][0])
    wB, hB = faceB[0] - 2 * SETBACK, faceB[1] - 2 * SETBACK
    r_corner = (wB - pitch_x) / 2
    plate_b = extrude(Plane(origin=oB, x_dir=xB, z_dir=nB)
                      * RectangleRounded(wB, hB, r_corner), T_PLATE)

    # --- beam -------------------------------------------------------------
    # Straight box beam, prismatic along z.  Its plan outline is tied to the
    # mid planes of the two plates; its height band lies between the screw
    # rows of both patterns, and its flanks stay off the two side screws of A.
    cA = oA + nA * (T_PLATE / 2)
    cB = oB + nB * (T_PLATE / 2)
    plan = [cA + uA * A_LAT[0], cA + uA * A_LAT[1],
            cB + xB * B_LAT[0], cB + xB * B_LAT[1]]
    plan = [Vector(v.X, v.Y, BEAM_Z[0]) for v in plan]
    web = extrude(Face(Wire.make_polygon(plan, close=True)),
                  BEAM_Z[1] - BEAM_Z[0], dir=(0, 0, 1))
    long_edges = [ed for ed in web.edges()
                  if abs((ed @ 1 - ed @ 0).normalized().dot(Z)) < 0.5
                  and abs((ed @ 1 - ed @ 0).normalized().dot(nA)) > 0.7]
    assert len(long_edges) == 4, len(long_edges)
    web = fillet(long_edges, R_BEAM)

    body = plate_a + web + plate_b
    body = body.clean()

    # --- root fillets and rim chamfers -----------------------------------
    def on_front(ed, o, n):
        return all(abs((ed @ t - o).dot(n) - T_PLATE) < 1e-4 for t in (0, 0.3, 0.7, 1))

    def radial(ed, o, n):
        v = (ed @ 0.37) - o
        return (v - n * v.dot(n)).length

    def is_outline(ed):
        if on_front(ed, oA, nA):
            return radial(ed, oA, nA) > rA - 0.5
        if on_front(ed, oB, nB):
            lat = (ed @ 0.37) - oB
            return (abs(lat.dot(xB)) > wB / 2 - r_corner * 0.31
                    or abs(lat.dot(yB)) > hB / 2 - r_corner * 0.31)
        return False

    roots = [ed for ed in body.edges()
             if (on_front(ed, oA, nA) or on_front(ed, oB, nB)) and not is_outline(ed)]
    body = fillet(roots, R_ROOT)
    rims = [ed for ed in body.edges() if is_outline(ed)]
    body = chamfer(rims, C_RIM)

    # --- screw holes ------------------------------------------------------
    for b in BOLTS:
        pos, ax = Vector(*b["position"]), Vector(*b["axis"])
        hole = extrude(Plane(origin=pos - ax * 1, z_dir=ax) * Circle(HOLE[b["size"]] / 2),
                       T_PLATE + 2)
        body = body - hole
    return body


if __name__ == "__main__":
    part = build()
    solids = part.solids()
    assert len(solids) == 1 and part.is_valid, (len(solids), part.is_valid)
    export_step(solids[0], str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(solids[0].volume), "mm3",
          "faces", len(solids[0].faces()))
