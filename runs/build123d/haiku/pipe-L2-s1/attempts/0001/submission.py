"""Connector for pipe-L2-s1: saddle pad on the 68 mm pipe, flat plate on the
38 mm square mount, joined by a tapered web. Build with build123d.

    python submission.py        # writes output.step next to this file
"""
import json
import math
import pathlib

from build123d import Plane, Solid, Vector, Wire, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACE = POSE["mounting_faces"][0]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
PIPE_PLACE = TASK["poses"][0]["placements"]["A"]

# ---- design numbers -------------------------------------------------------
PIPE_R = 34.0          # pipe outside radius (seat is the pipe surface itself)
PAD_OUT_R = 42.0       # pad outer face: grip 8 mm for the M5 bolts (3.75-25)
PAD_HALF_LEN = 25.5    # pad spans the two bolt rows at +-17.5 mm, plus boss
PAD_EDGE_MARGIN = 8.0  # boss margin beyond the bolt centres, along the arc
PLATE_T = 8.0          # plate thickness = grip for the M4 bolts (3-25)
PLATE_HALF = 17.0      # 34 mm square: inside the 38 mm mount by 2 mm
WEB_PAD_W, WEB_PAD_H = 18.0, 10.0
WEB_PLATE_W, WEB_PLATE_H = 26.0, 10.0
HOLE_D = {"M5": 5.5, "M4": 4.5}  # middle of the clearance ranges


def v(p):
    return Vector(*p)


def perp_plane(origin, z_dir, x_hint=None):
    z = z_dir.normalized()
    ref = x_hint if x_hint is not None else (Vector(1, 0, 0) if abs(z.X) < 0.9 else Vector(0, 1, 0))
    x = (ref - z * (ref.dot(z))).normalized()
    return Plane(origin, x, z)


def build():
    # ---- pipe frame: Z along the pipe axis, X towards the bolt mid-line ----
    cols = [[PIPE_PLACE["rotation"][r][c] for r in range(3)] for c in range(3)]
    P0 = v(PIPE_PLACE["origin"])
    Zp = v(cols[0]).normalized()                       # pipe axis (rotation column 0)
    a1 = v(BOLTS["A1"]["axis"])
    a3 = v(BOLTS["A3"]["axis"])
    m = (a1 + a3)
    X = (m - Zp * m.dot(Zp)).normalized()              # radial mid-line of the bolt pairs
    Y = Zp.cross(X)

    def angle_deg(axis):
        return math.degrees(math.atan2(axis.dot(Y), axis.dot(X)))

    half_arc = max(abs(angle_deg(v(BOLTS[k]["axis"]))) for k in ("A1", "A2", "A3", "A4"))
    half_arc += math.degrees(PAD_EDGE_MARGIN / PIPE_R)

    # ---- saddle pad: sector of the pipe's boss ring, inner face on the pipe --
    def rot_about_z(vec, deg):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        return vec * c + Zp.cross(vec) * s

    sector_plane = Plane(P0 - Zp * PAD_HALF_LEN, rot_about_z(X, -half_arc), Zp)
    outer = Solid.make_cylinder(PAD_OUT_R, 2 * PAD_HALF_LEN, sector_plane, 2 * half_arc)
    inner = Solid.make_cylinder(PIPE_R, 2 * PAD_HALF_LEN + 2,
                                Plane(P0 - Zp * (PAD_HALF_LEN + 1), X, Zp))
    pad = outer - inner

    # ---- flat plate on the 38 mm square mount ------------------------------
    oB = v(FACE["origin"])
    nb = v(FACE["normal"]).normalized()
    t = (Vector(0, 0, 1).cross(nb)).normalized()       # in-plane axis of the face
    w = nb.cross(t).normalized()
    plate_plane = Plane(oB - t * PLATE_HALF - w * PLATE_HALF, t, nb)
    plate = Solid.make_box(2 * PLATE_HALF, 2 * PLATE_HALF, PLATE_T, plate_plane)

    # ---- web joining pad and plate ----------------------------------------
    Pe = P0 + X * (PIPE_R + 4.0)                       # centre of the pad web end
    Qe = oB + nb * 4.0                                 # inside the plate
    u = (Qe - Pe).normalized()
    w1 = u.cross(Zp).normalized()
    v1 = u.cross(w1).normalized()

    def rect(center, width, height):
        pts = [center + w1 * (sx * width / 2) + v1 * (sy * height / 2)
               for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        return Wire.make_polygon(pts, close=True)

    web = Solid.make_loft([rect(Pe, WEB_PAD_W, WEB_PAD_H), rect(Qe, WEB_PLATE_W, WEB_PLATE_H)],
                          ruled=True)

    # keep the web on the free side of the mounting face (never below it)
    above_face = Solid.make_box(400, 400, 400, Plane(oB - t * 200 - w * 200, t, nb))
    web = web & above_face
    body = pad.fuse(plate, web).clean()

    # ---- clearance holes on every bolt axis ------------------------------
    for bid in sorted(BOLTS):
        b = BOLTS[bid]
        pos, ax = v(b["position"]), v(b["axis"]).normalized()
        dia = HOLE_D[b["size"]]
        start = pos - ax * (6.0 if b["size"] == "M5" else 2.0)
        length = 20.0 if b["size"] == "M5" else 12.0
        body = body - Solid.make_cylinder(dia / 2, length, perp_plane(start, ax))

    return body.clean()


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "valid:", part.is_valid, "solids:", len(part.solids()), "vol:", round(part.volume, 1))
