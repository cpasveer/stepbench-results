"""Connector between fixed flange A (6x M5) and fixed block B (4x M6).

Design: two flat pads, one bolted to each mounting face, joined by a vertical
bridging wall that runs through the gap between the two bolt clusters, so the
key-turning corridors of all ten bolts stay open. Every surface is vertical or
horizontal-up-facing, so the part prints upright without support.

    python submission.py                 # writes output.step
"""
import json
import math
import pathlib

from build123d import (BuildPart, BuildSketch, Circle, Locations, Plane, Polygon,
                       Rectangle, Solid, add, export_step, extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = {b["id"]: b for b in POSE["bolts"]}

PAD_A_T = 8.0          # grip under the M5 heads (3.75..25)
PAD_B_T = 8.0          # grip under the M6 heads (4.5..25)
HOLE_A = 5.5           # clearance for M5 (5.3..5.8)
HOLE_B = 6.6           # clearance for M6 (6.4..7.0)
RING_A = 6.0           # pad footprint radius around each M5 bolt
RING_B = 6.0
WALL_T = 8.0           # bridge wall thickness
Z_BOT, Z_TOP = -64.0, 28.0   # bridge height span (B pad limits)


def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def vadd(a, b): return tuple(x + y for x, y in zip(a, b))
def mul(a, s): return tuple(x * s for x in a)


def face_frame(name):
    f = FACES[name]
    o = tuple(f["origin"])
    n = tuple(f["normal"])
    x = (-n[1], n[0], 0.0)        # horizontal tangent; y = n x x is vertical
    return o, n, x


def local_uv(name, p):
    o, n, x = face_frame(name)
    d = sub(p, o)
    return dot(d, x), d[2]        # (u along tangent, v vertical)


def plane_for(name):
    o, n, x = face_frame(name)
    return Plane(origin=o, x_dir=x, z_dir=n)


def bolt_uv(bid):
    b = BOLTS[bid]
    name = bid[0]
    return local_uv(name, tuple(b["position"]))


def cyl(p0, direction, r, h):
    return Solid.make_cylinder(r, h, Plane(origin=p0, z_dir=direction))


def strip_prism(poly_xy, z0, z1):
    with BuildPart() as bp:
        with BuildSketch(Plane(origin=(0, 0, z0))):
            Polygon(*poly_xy)
        extrude(amount=z1 - z0)
    return bp.part


def build():
    oA, nA, tA = face_frame("A")
    oB, nB, sB = face_frame("B")

    # ---- pad on face A: footprint = six bolt bosses joined through the bolt
    # centres, with a steep (52 deg) tent cut under the two lower bolts so the
    # underside never overhangs more than 38 deg when printed upright.
    a_pts = [bolt_uv(b) for b in ("A1", "A6", "A5", "A4", "A3", "A2")]
    with BuildSketch() as hull_a:
        Polygon(*a_pts)
        with Locations(*a_pts):
            Circle(RING_A)
    v_low = min(v for _, v in a_pts) - RING_A          # lowest ring point
    tent = [(-45.0, v_low + 1.3 * 30.0), (-15.0, v_low), (0.0, v_low + 1.3 * 15.0),
            (15.0, v_low), (45.0, v_low + 1.3 * 30.0), (45.0, 60.0), (-45.0, 60.0)]
    with BuildSketch() as above_a:
        Polygon(*tent)
    with BuildPart() as pa:
        with BuildSketch(plane_for("A")):
            add(hull_a.sketch & above_a.sketch)
        extrude(amount=PAD_A_T)
    padA = pa.part

    # ---- pad on face B: rounded rectangle around the 2 x 2 bolt grid -------
    with BuildPart() as pb:
        with BuildSketch(plane_for("B")):
            Rectangle(42.0, 92.0)
        extrude(amount=PAD_B_T)
    padB = pb.part

    # ---- bridging wall: polyline in the horizontal plane, extruded in z ----
    # horizontal coords: d = along nA from face A, t = along tA
    def to_xy(d, t):
        return (oA[0] + d * nA[0] + t * tA[0], oA[1] + d * nA[1] + t * tA[1])

    # centre of the B pad, a few mm inside it
    cB = vadd(oB, mul(nB, 4.0))
    dB = dot(sub(cB, oA), nA)
    tB = dot(sub(cB, oA), tA)
    path = [(PAD_A_T, 0.0), (30.0, 0.0), (dB, tB)]   # starts on the A pad's back face
    hw = WALL_T / 2
    # miter offsets for the polyline
    segs = []
    for (d0, t0), (d1, t1) in zip(path[:-1], path[1:]):
        L = math.hypot(d1 - d0, t1 - t0)
        segs.append(((d1 - d0) / L, (t1 - t0) / L))
    normals = [(-u[1], u[0]) for u in segs]  # left normals (d,t)
    left, right = [], []
    for i, (d, t) in enumerate(path):
        if i == 0:
            nl = normals[0]; m = (nl[0] * hw, nl[1] * hw)
        elif i == len(path) - 1:
            nl = normals[-1]; m = (nl[0] * hw, nl[1] * hw)
        else:
            n1, n2 = normals[i - 1], normals[i]
            k = 1.0 + (n1[0] * n2[0] + n1[1] * n2[1])
            m = (((n1[0] + n2[0]) / k) * hw, ((n1[1] + n2[1]) / k) * hw)
        left.append((d + m[0], t + m[1]))
        right.append((d - m[0], t - m[1]))
    outline_dt = left + right[::-1]
    outline_xy = [to_xy(d, t) for d, t in outline_dt]
    wall = strip_prism(outline_xy, Z_BOT, Z_TOP)

    body = padA + padB + wall

    # ---- through holes on every bolt axis ------------------------------
    for bid in BOLTS:
        b = BOLTS[bid]
        p = tuple(b["position"])
        ax = tuple(b["axis"])
        r = (HOLE_A if bid[0] == "A" else HOLE_B) / 2.0   # diameters -> radii
        # start 1 mm behind the seat, run through the pad
        p0 = sub(p, mul(ax, 1.0))
        h = (PAD_A_T if bid[0] == "A" else PAD_B_T) + 2.0
        body = body - cyl(p0, ax, r, h)

    return body


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "valid:", part.is_valid, "volume:", round(part.volume, 1))
