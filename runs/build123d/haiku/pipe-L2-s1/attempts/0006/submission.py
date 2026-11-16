"""Connector for pipe-L2-s1 (build123d).

A saddle pad sits on the 68 mm pipe, a flat plate sits on the 38 mm square
mount, and a web joins them. The part hangs from the mount and is printed
as it sits (Z up): the pad's underside and the plate's lower edge are
chamfered, and the web is steep, so nothing hangs out past 45 degrees.

    python submission.py        # writes output.step next to this file
"""
import json
import math
import pathlib

from build123d import Axis, Face, Plane, Solid, Vector, Wire, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACE = POSE["mounting_faces"][0]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
PIPE_PLACE = TASK["poses"][0]["placements"]["A"]

# ---- design numbers (mm) ---------------------------------------------------
PIPE_R = 34.0           # seat is the pipe surface itself (concentric r = 34)
PAD_OUT_R = 42.0        # pad outer face: 8 mm grip for the M5 bolts (3.75-25)
PAD_S_TOP = 25.5        # pad top end, 8 mm beyond the upper bolt row (s = +17.5)
PAD_S_BOT = -31.0       # pad bottom end at the pipe; chamfered up to the outside
PAD_ARC_MARGIN = 8.0    # boss margin past the outer bolts, measured on the arc
PLATE_T = 8.0           # plate thickness = 8 mm grip for the M4 bolts (3-25)
PLATE_HALF = 17.0       # 34 mm square on the 38 mm mount, 2 mm inside the edge
PLATE_BEVEL = (8.0, 9.0)  # lower outer edge bevel: 48 deg from horizontal (run 8, rise 9)
WEB_PAD_W, WEB_PAD_H = 10.0, 7.0
WEB_PAD_S = -24.0       # where the web meets the pad (along the pipe axis)
WEB_PAD_THETA = 0.0    # ... at this angle round the pipe
WEB_PAD_R = 36.0        # ... and at this radius (inside the pad wall 34..42)
WEB_PLATE_DT = 0.0     # web root on the plate's bevel, offset from the mount centre
WEB_PLATE_BETA = 6.0    # ... this far out from the seat, measured across the plate
JOINT_R = 2.0           # fillets where the web meets the plate and the pad
HOLE_D = {"M5": 5.75, "M4": 4.7}  # inside the clearance ranges, clear of the shank


def v(p):
    return Vector(*p)


def perp_plane(origin, z_dir):
    z = z_dir.normalized()
    ref = Vector(1, 0, 0) if abs(z.X) < 0.9 else Vector(0, 1, 0)
    return Plane(origin, (ref - z * ref.dot(z)).normalized(), z)


def half_space(origin, normal, lateral_a, lateral_b):
    """Solid occupying the side of the plane through origin that normal points to."""
    n = normal.normalized()
    ydir = n.cross(lateral_a).normalized()
    base = origin - lateral_a * 200 - ydir * 200
    return Solid.make_box(400, 400, 200, Plane(base, lateral_a, n))


def build():
    # ---- pipe frame: Z along the pipe axis, X towards the bolt mid-line -----
    P0 = v(PIPE_PLACE["origin"])
    Zp = v([PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
    m = v(BOLTS["A1"]["axis"]) + v(BOLTS["A3"]["axis"])
    X = (m - Zp * m.dot(Zp)).normalized()
    Y = Zp.cross(X)

    def angle_deg(axis):
        return math.degrees(math.atan2(axis.dot(Y), axis.dot(X)))

    half_arc = max(abs(angle_deg(v(BOLTS[k]["axis"]))) for k in ("A1", "A2", "A3", "A4"))
    half_arc += math.degrees(PAD_ARC_MARGIN / PIPE_R)

    # ---- saddle pad -------------------------------------------------------
    def rot_about_z(vec, deg):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        return vec * c + Zp.cross(vec) * s

    # the pad is a revolved section: r 34..42, underside chamfered at 45 deg
    def at_angle(radius, s_pos, deg):
        return P0 + rot_about_z(X, deg) * radius + Zp * s_pos

    corners = [at_angle(PIPE_R, PAD_S_BOT, -half_arc),
               at_angle(PAD_OUT_R, PAD_S_BOT + (PAD_OUT_R - PIPE_R), -half_arc),
               at_angle(PAD_OUT_R, PAD_S_TOP, -half_arc),
               at_angle(PIPE_R, PAD_S_TOP, -half_arc)]
    section = Face(Wire.make_polygon(corners, close=True))
    pad = Solid.revolve(section, 2 * half_arc, Axis(P0, Zp))

    # ---- flat plate on the 38 mm square mount ------------------------------
    oB = v(FACE["origin"])
    nb = v(FACE["normal"]).normalized()
    t = Vector(0, 0, 1).cross(nb).normalized()        # in-plane, horizontal
    w = nb.cross(t).normalized()                      # in-plane, vertical
    plate_plane = Plane(oB - t * PLATE_HALF - w * PLATE_HALF, t, nb)
    plate = Solid.make_box(2 * PLATE_HALF, 2 * PLATE_HALF, PLATE_T, plate_plane)
    # bevel the lower outer edge (48 deg face): no horizontal ceiling under the plate
    bt, bz = PLATE_BEVEL
    bev_n = (nb * bz - Vector(0, 0, 1) * bt).normalized()     # points down and out
    bev_origin = oB - w * PLATE_HALF                           # seat corner, bottom edge
    plate = plate - half_space(bev_origin, bev_n, t, None)

    # ---- web hanging from the mount down to the pad -----------------------
    Pe = at_angle(WEB_PAD_R, WEB_PAD_S, WEB_PAD_THETA)
    # root on the sloped underside of the plate (joins it at about 70 deg)
    z_bevel = oB.Z - PLATE_HALF + WEB_PLATE_BETA * PLATE_BEVEL[1] / PLATE_BEVEL[0]
    Qe = oB + nb * WEB_PLATE_BETA + t * WEB_PLATE_DT + Vector(0, 0, z_bevel - oB.Z)
    u = (Pe - Qe).normalized()
    w1 = u.cross(Vector(0, 0, 1)).normalized()
    v1 = u.cross(w1).normalized()

    # the web is a straight bar from the plate root Qe to the pad point Pe (2 mm into the pad)
    length = (Pe - Qe).length
    bar_w, bar_h = WEB_PAD_W, WEB_PAD_H
    bar_origin = Qe - u * 2.0 - w1 * (bar_w / 2) - v1 * (bar_h / 2)
    web = Solid.make_box(bar_w, bar_h, length + 2.0, Plane(bar_origin, w1, u))

    body = pad.fuse(plate, web).clean()
    # nothing goes behind the mounting face (the fixed part is there)
    body = body - half_space(oB, -nb, t, None)
    # keep everything outside the pipe (the pad already bears on it at r = 34)
    body = body - Solid.make_cylinder(PIPE_R, 400, Plane(P0 - Zp * 200, X, Zp))

    # ---- clearance holes on every bolt axis -------------------------------
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
    print("wrote", HERE / "output.step", "valid:", part.is_valid, "solids:", len(part.solids()),
          "vol:", round(part.volume, 1))
