"""Connector between round flange A (6x M5) and rectangular block B (4x M6).

Printed in PLA standing upright (world +Z is the build direction, the part's
flat underside at z = -64 is the bed face), so every load-bearing wall is
vertical and no support is needed:

* plate A  - a "tombstone" plate parallel to face A: round top concentric with
  the flange, straight sides running down to the bed.  It seats on the flange
  through a 1 mm raised pad (r = 35, 1 mm inside the 74.5 mm face) whose edge
  is drafted at 55 deg so its lower rim is self-supporting.  The rest of the
  plate stands 1 mm off the flange face.
* plate B  - a 44 x 92.5 mm plate parallel to face B, 1 mm inside the 46 x 96
  face on all seated edges.
* web      - one 10 mm vertical wall from plate to plate, routed between the
  lower A bolts and between the B bolt columns so every socket-head screw can
  be driven with an ordinary L-key swung outward.
"""
import json
import math
import pathlib

from build123d import (
    Align, Axis, Circle, Cylinder, Location, Part, Plane, Polygon, Rectangle,
    Vector, export_step, extrude, fillet, Edge
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

Z_BED = -64.0          # underside of the part (B face bottom is -66)
T_A = 8.0              # plate A total thickness (pad + plate)
T_B = 8.0              # plate B thickness
PAD = 1.0              # seat pad height on A
R_PAD = 35.0           # pad radius (face radius 37.25, 1 mm setback -> 36.25)
R_TOMB = 38.0          # plate A outline radius
PAD_DRAFT = 55.0       # pad edge draft (deg from the pad axis): 35 deg overhang
B_HALF_W = 22.0        # plate B half width (face 23)
Z_B_TOP = 28.5         # plate B top (face top 30, setback -> 29)
WEB_T = 9.0
HOLE = {"M5": 5.5, "M6": 6.6}


def V(p):
    return Vector(*p)


def face_plane(name: str) -> Plane:
    f = FACES[name]
    n = V(f["normal"]).normalized()
    x = Vector(-n.Y, n.X, 0).normalized()     # horizontal, so local y = world +Z
    return Plane(origin=V(f["origin"]), x_dir=x, z_dir=n)


def build() -> Part:
    pa, pb = face_plane("A"), face_plane("B")
    ya_bed = Z_BED - pa.origin.Z
    yb_bed = Z_BED - pb.origin.Z
    yb_top = Z_B_TOP - pb.origin.Z

    # ---- plate A (tombstone) ------------------------------------------------
    pa1 = pa.offset(PAD)
    tomb = Circle(R_TOMB) + Rectangle(2 * R_TOMB, -ya_bed, align=(Align.CENTER, Align.MAX))
    plate_a = extrude(pa1 * tomb, amount=T_A - PAD)
    # seat pad: drafted 45 deg towards the flange
    pad = extrude(Plane(pa1.origin, pa1.x_dir, -pa1.z_dir) * Circle(R_PAD + PAD * math.tan(math.radians(PAD_DRAFT))),
                  amount=PAD, taper=PAD_DRAFT)

    # ---- plate B ---------------------------------------------------------------
    rect_b = Rectangle(2 * B_HALF_W, yb_top - yb_bed, align=(Align.CENTER, Align.MIN))
    plate_b = extrude(pb * Location((0, yb_bed)) * rect_b, amount=T_B)

    # ---- web ---------------------------------------------------------------
    # end points (plan) inside each plate, offset sideways from the centres
    ua, ub = pa.x_dir, pb.x_dir          # horizontal in-plane directions
    end_a = pa.origin + pa.z_dir * (T_A * 0.55) + ua * (-2.3)
    end_b = pb.origin + pb.z_dir * (T_B * 0.55) + ub * (1.0)
    end_a = Vector(end_a.X, end_a.Y, Z_BED)
    end_b = Vector(end_b.X, end_b.Y, Z_BED)
    run = end_b - end_a
    L = run.length
    dirw = run.normalized()
    nw = Vector(dirw.Y, -dirw.X, 0)   # so the web plane local y = +Z
    wp = Plane(origin=end_a, x_dir=dirw, z_dir=nw)
    z_top_a, z_top_b = 34.0, 20.0
    prof = Polygon((0, 0), (L, 0), (L, z_top_b - Z_BED), (0, z_top_a - Z_BED), align=None)
    web = extrude(wp * prof, amount=WEB_T / 2, both=True)

    part = plate_a + pad + plate_b + web

    # fillet the vertical web/plate junctions and plate B's free vertical corners
    def vertical(e):
        d = (e.end_point() - e.start_point())
        return d.length > 5 and abs(d.normalized().Z) > 0.999
    cand = [e for e in part.edges() if vertical(e)]
    jn = []
    for e in cand:
        m = e.center()
        # web/plate junction edges lie on the web side faces near either plate
        dA = (m - pa.origin).dot(pa.z_dir)
        dB = (m - pb.origin).dot(pb.z_dir)
        if abs(dA - T_A) < 0.2 or abs(dB - T_B) < 0.2:
            off = abs((m - end_a).dot(nw))
            if abs(off - WEB_T / 2) < 2.0:
                jn.append(e)
    part = fillet(jn, 3.5)

    # round plate B's four vertical corners
    def on_b_corner(e):
        if not vertical(e):
            return False
        m = e.center() - pb.origin
        return abs(abs(m.dot(pb.x_dir)) - B_HALF_W) < 0.05
    part = fillet([e for e in part.edges() if on_b_corner(e)], 3.0)

    # ---- bolt holes -------------------------------------------------------------
    for b in BOLTS:
        ax = V(b["axis"]).normalized()
        p0 = V(b["position"]) - ax * 2
        cyl = Cylinder(HOLE[b["size"]] / 2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN))
        cyl = Plane(origin=p0, z_dir=ax) * cyl
        part = part - cyl
    return part


if __name__ == "__main__":
    p = build()
    if isinstance(p, Part) is False:
        p = Part(p)
    print("solids", len(p.solids()), "volume", round(p.volume), "valid", p.is_valid)
    print(p.bounding_box())
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
