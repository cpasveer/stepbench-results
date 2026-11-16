"""Connector for task flange-L2-s1 (build123d).

Concept: two 8 mm bolting plates joined by one thick central web, the whole
part being (nearly) a prism along world Z so that it prints without support
standing on its top face (world +Z end on the bed, i.e. printed upside down).

  * A pad  : the round flange seat, a disc cut flat at the bed and with two
             45 degree flanks so that it grows from the bed without overhang.
  * B plate: the rectangular seat; above the block's face it runs on to the
             bed, set back 2 mm behind a 45 degree chamfer so it does not seat
             beyond the mounting face.
  * web    : a solid spine between the two bolt patterns.  In plan it stays
             inside the corridor left free by the bolt heads and the hex keys
             (all keys work straight, long leg pointing outward), flares
             towards B, and its free end is cut on a slope from the A pad
             down to the foot of the B plate.
"""
import json
import math
import pathlib

from build123d import (Axis, BuildPart, BuildSketch, Circle, Cylinder, Location, Mode,
                       Plane, Polygon, Pos, Rectangle, Vector, add, export_step,
                       extrude, fillet)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]
BOLTS = POSE["bolts"]

# ---- parameters ------------------------------------------------------------
T_PLATE = 8.0        # plate thickness = grip of every bolt
R_PAD = 36.0         # A pad radius (face 37.25 - setback)
PAD_FLAT = 30.25     # height of the pad's flat (bed) edge above the A centre
B_HALF_W = 22.0      # B plate half width  (face 23 - 1)
B_BOTTOM = -47.0     # B plate bottom edge, rel. to B centre (face -48 + 1)
B_SEAT_TOP = 47.0    # B seat top edge, rel. to B centre
B_RELIEF = 2.0       # set-back of the B plate above the seat
D_A = 5.5            # M5 clearance hole
D_B = 6.6            # M6 clearance hole
WEB_A = (8.0, -6.5)  # web roots on the A plate (lateral position)
WEB_KINK = (-11.0, 38.0)
WEB_B = (7.5, -6.5)  # web roots on the B plate (B lateral position)
R_ROOT = 2.0
R_KINK = 10.0
WEB_Z_A = -32.5      # web end height at the A plate, rel. to A centre


def build():
    a, b = FACES
    oa, na = Vector(*a["origin"]), Vector(*a["normal"]).normalized()
    ob, nb = Vector(*b["origin"]), Vector(*b["normal"]).normalized()
    zh = Vector(0, 0, 1)
    ta = na.cross(zh).normalized()           # A lateral direction (horizontal)
    # local frame: x = ta, y = na, z = world z, origin at A centre
    frame = Plane(origin=oa, x_dir=ta, z_dir=zh)

    def loc(v):   # world point -> local
        d = v - oa
        return Vector(d.dot(ta), d.dot(na), d.dot(zh))

    def locd(v):  # world direction -> local
        return Vector(v.dot(ta), v.dot(na), v.dot(zh))

    bc = loc(ob)                              # B centre, local
    bn = locd(nb)                             # B normal (towards the part)
    bt = zh.cross(bn).normalized()            # B lateral direction, local
    z_bed = PAD_FLAT                          # local z of the bed plane
    z_bot = bc.Z + B_BOTTOM

    def bpt(tb, d):                           # plan point from B coordinates
        p = Vector(bc.X, bc.Y, 0) + bt * tb + bn * d
        return (p.X, p.Y)

    # ---- A pad ---------------------------------------------------------------
    c45 = PAD_FLAT + math.sqrt(R_PAD**2 - PAD_FLAT**2)
    with BuildPart() as pad:
        with BuildSketch(Plane(origin=(0, 0, 0), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))):
            Circle(R_PAD)
            Polygon((-(c45 - PAD_FLAT), PAD_FLAT), (c45 - PAD_FLAT, PAD_FLAT),
                    (c45 + 60, -60), (-(c45 + 60), -60), align=None, mode=Mode.INTERSECT)
        extrude(amount=T_PLATE)

    # ---- B plate -------------------------------------------------------------
    pl_b = Plane(origin=(bc.X, bc.Y, bc.Z), x_dir=(bt.X, bt.Y, 0), z_dir=(bn.X, bn.Y, 0))
    if pl_b.y_dir.Z < 0:
        pl_b = Plane(origin=(bc.X, bc.Y, bc.Z), x_dir=(-bt.X, -bt.Y, 0), z_dir=(bn.X, bn.Y, 0))
    top_rel = z_bed - bc.Z
    with BuildPart() as plate:
        with BuildSketch(pl_b) as sk:
            Polygon((-B_HALF_W, B_BOTTOM), (B_HALF_W, B_BOTTOM), (B_HALF_W, top_rel),
                    (-B_HALF_W, top_rel), align=None)
            fillet(sk.vertices().sort_by(Axis.Y)[:2], 5.0)
        extrude(amount=T_PLATE)
        # relief above the seat: 45 degree chamfer, then 2 mm set-back up to the bed
        side = Plane(origin=(bc.X, bc.Y, bc.Z), x_dir=(bn.X, bn.Y, 0), z_dir=(-pl_b.x_dir.X, -pl_b.x_dir.Y, 0))
        sgn = 1 if side.y_dir.Z > 0 else -1
        with BuildSketch(side):
            Polygon(*[(d, sgn * z) for d, z in [(-1, B_SEAT_TOP), (0, B_SEAT_TOP),
                                              (B_RELIEF, B_SEAT_TOP + B_RELIEF),
                                              (B_RELIEF, top_rel + 1), (-1, top_rel + 1)]], align=None)
        extrude(amount=B_HALF_W + 1, both=True, mode=Mode.SUBTRACT)

    # ---- web -----------------------------------------------------------------
    w1b = bpt(WEB_B[0], T_PLATE)
    w2b = bpt(WEB_B[1], T_PLATE)
    pts = [(14, 3), (-14, 3), (-14, T_PLATE), (WEB_A[1], T_PLATE), WEB_KINK, w2b,
           bpt(-14, T_PLATE), bpt(-14, 3), bpt(14, 3), bpt(14, T_PLATE), w1b,
           (WEB_A[0], T_PLATE), (14, T_PLATE)]
    roots = [(WEB_A[1], T_PLATE), w2b, w1b, (WEB_A[0], T_PLATE)]
    slope = (WEB_Z_A - (z_bot + 1.5)) / (w1b[1] - T_PLATE)

    def zcut(n):
        return WEB_Z_A - slope * (n - T_PLATE)

    with BuildPart() as web:
        with BuildSketch(Plane.XY.offset(z_bot + 0.5)) as ws:
            Polygon(*pts, align=None)

            def near(p):
                return min(ws.vertices(), key=lambda v: (v.X - p[0])**2 + (v.Y - p[1])**2)
            fillet([near(p) for p in roots], R_ROOT)
            fillet([near(WEB_KINK)], R_KINK)
        extrude(amount=z_bed - (z_bot + 0.5))
        # sloped free end
        with BuildSketch(Plane(origin=(0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))):
            Polygon((-20, zcut(-20)), (100, zcut(100)), (100, -200), (-20, -200), align=None)
        extrude(amount=150, both=True, mode=Mode.SUBTRACT)

    for nm, q in (("pad", pad), ("plate", plate), ("web", web)):
        bb = q.part.bounding_box(); print(nm, bb.min, bb.max, q.part.volume)
    print("bc", bc, "bn", bn, "bt", bt)
    with BuildPart() as part:
        add(pad.part)
        add(plate.part)
        add(web.part)
        # bolt holes
        for bolt in BOLTS:
            p = loc(Vector(*bolt["position"]))
            ax = locd(Vector(*bolt["axis"]))
            dia = D_A if bolt["size"] == "M5" else D_B
            hp = Plane(origin=(p.X, p.Y, p.Z), z_dir=(ax.X, ax.Y, ax.Z))
            with BuildSketch(hp):
                Circle(dia / 2)
            extrude(amount=40, both=True, mode=Mode.SUBTRACT)

    return frame.location * part.part


if __name__ == "__main__":
    result = build()
    export_step(result, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
