"""Pipe-to-flat-mount bracket, task pipe-L2-s1 (FreeCAD script).

    freecadcmd submission.py        # writes output.step

Design, in one paragraph
------------------------
The part is printed standing on one end of the pipe saddle, i.e. the build
direction is the pipe axis.  Everything that belongs to the pipe is then a
plain prism with vertical walls: a V-block saddle (bore R34, one flat per bolt
row, square to the bolts) and a 10 mm web that leaves the V between the two
bolt rows.  The pad for mount B leans 40 deg off the build direction, which
prints without support; it runs on down to the build plate so that it starts
on the bed, and its back is relieved below the lower bolt row so the part only
touches B inside B's own outline.  The web's top edge slopes from the saddle
down to the pad like a gusset.

Frames
------
F : origin on the pipe axis, X = "u" (square to the pipe, in the plane of the
    pad's tilt), Y = "w" (the pad's horizontal edge), Z = pipe axis.
B : the mount's own frame, X = "c", Y = "w", Z = "n" (outward normal).
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]
V = App.Vector

# ----------------------------------------------------------------- parameters
R_PIPE = 34.0
SADDLE_HALF = 27.0     # saddle half length along the pipe (bolts at +-17.5)
FLAT_DIST = 41.0       # bolt seat flats: 7 mm of grip under the M5 heads
FLAT_OUT = 9.0         # flat width outboard of the bolt
D_A = 5.5              # M5 clearance
PAD_T = 5.0            # pad thickness = M4 grip
PAD_HALF = 18.0        # pad half width: 1 mm inside B's 38 mm face
D_B = 4.5              # M4 clearance
PAD_CORNER = 5.0
WEB_HALF = 5.0         # web is 10 mm thick
WEB_FILLET = 2.5       # web to pad
ROOT_FILLET = 3.0      # web to saddle flats
SEAT_END = -16.4       # the seat stops here (lower bolt row is at c = -12.5)
RELIEF_R = 15.0
RELIEF_SLOPE = 44.0    # steepest overhang of the relieved back, deg
WEB_TOP_AT_PAD = 14.0  # c where the web's top edge lands on the pad
WEB_TOP_AT_SADDLE = 24.0


def col(rows, j):
    return V(rows[0][j], rows[1][j], rows[2][j])


RA = POSE["placements"]["A"]["rotation"]
RB = POSE["placements"]["B"]["rotation"]
OA = V(*POSE["placements"]["A"]["origin"])
OB = V(*POSE["placements"]["B"]["origin"])
AX = col(RA, 0)                       # pipe axis
BC, BW, BN = col(RB, 0), col(RB, 1), col(RB, 2)
assert abs(BW.dot(AX)) < 1e-9, "pad edge is expected square to the pipe axis"
EW = BW
EU = EW.cross(AX)
assert BN.dot(AX) > 0 and BC.dot(AX) > 0


def matrix(ex, ey, ez, o):
    return App.Matrix(ex.x, ey.x, ez.x, o.x,
                      ex.y, ey.y, ez.y, o.y,
                      ex.z, ey.z, ez.z, o.z,
                      0, 0, 0, 1)


M_F = matrix(EU, EW, AX, OA)
M_B = matrix(BC, BW, BN, OB)


def to_f(p):
    d = p - OA
    return V(d.dot(EU), d.dot(EW), d.dot(AX))


def from_b(c, w, n):
    return OB + BC * c + BW * w + BN * n


# ------------------------------------------------------------------ 2D helper
def outline(pts, radii, lift, closing_arc_mid=None):
    """Closed wire through 2D pts; corner i rounded with radii[i] (0 = sharp).
    If closing_arc_mid is given, the last->first segment is an arc through it.
    lift maps (a, b) to a 3D Vector."""
    n = len(pts)
    ins, outs, mids = [], [], []
    for i in range(n):
        p = pts[i]
        r = radii[i]
        if r <= 0:
            ins.append(p), outs.append(p), mids.append(None)
            continue
        a, b = pts[i - 1], pts[(i + 1) % n]
        d1 = (a[0] - p[0], a[1] - p[1])
        d2 = (b[0] - p[0], b[1] - p[1])
        l1, l2 = math.hypot(*d1), math.hypot(*d2)
        d1 = (d1[0] / l1, d1[1] / l1)
        d2 = (d2[0] / l2, d2[1] / l2)
        ang = math.acos(max(-1.0, min(1.0, d1[0] * d2[0] + d1[1] * d2[1])))
        t = r / math.tan(ang / 2)
        assert t < l1 and t < l2, ("fillet does not fit", i, t, l1, l2)
        bis = (d1[0] + d2[0], d1[1] + d2[1])
        lb = math.hypot(*bis)
        cdist = r / math.sin(ang / 2)
        cen = (p[0] + bis[0] / lb * cdist, p[1] + bis[1] / lb * cdist)
        ins.append((p[0] + d1[0] * t, p[1] + d1[1] * t))
        outs.append((p[0] + d2[0] * t, p[1] + d2[1] * t))
        mids.append((cen[0] - bis[0] / lb * r, cen[1] - bis[1] / lb * r))
    edges = []
    for i in range(n):
        if mids[i] is not None:
            edges.append(Part.Arc(lift(*ins[i]), lift(*mids[i]), lift(*outs[i])).toShape())
        j = (i + 1) % n
        if j == 0 and closing_arc_mid is not None:
            edges.append(Part.Arc(lift(*outs[i]), lift(*closing_arc_mid), lift(*ins[j])).toShape())
        else:
            edges.append(Part.LineSegment(lift(*outs[i]), lift(*ins[j])).toShape())
    return Part.Face(Part.Wire(edges))


def polar(r, ang):
    return (r * math.cos(ang), r * math.sin(ang))


def add(p, q, k=1.0):
    return (p[0] + q[0] * k, p[1] + q[1] * k)


# ------------------------------------------------------------ derived numbers
a_bolts = [b for b in BOLTS if b["id"].startswith("A")]
b_bolts = [b for b in BOLTS if b["id"].startswith("B")]
angs = sorted({round(math.atan2(to_f(V(*b["position"])).y, to_f(V(*b["position"])).x), 9)
               for b in a_bolts})
assert len(angs) == 2
TH3, TH1 = angs                       # the two bolt rows, as angles in F
assert 0 < TH1 - TH3 < math.pi / 2
bc = to_f(OB)                         # pad centre in F
WC = bc.y                             # the web's centre plane
nb = (BN.dot(EU), BN.dot(AX))         # pad normal in the (u, x) plane
cb = (BC.dot(EU), BC.dot(AX))
SEAT_OVERHANG = math.degrees(math.asin(nb[1]))


def pad_ux(c, n):
    return (bc.x + cb[0] * c + nb[0] * n, bc.z + cb[1] * c + nb[1] * n)


# web top plane: x = TOPX0 + TOPK * (u - TOPU0)
q_pad = pad_ux(WEB_TOP_AT_PAD, PAD_T)
r1, t1 = polar(1, TH1), (math.sin(TH1), -math.cos(TH1))   # t: towards the V apex
r3, t3 = polar(1, TH3), (-math.sin(TH3), math.cos(TH3))
P1, P3 = polar(FLAT_DIST, TH1), polar(FLAT_DIST, TH3)
ROOT1 = add(P1, t1, 15.0)             # where the web leaves the row-1 flat
ROOT3 = add(P3, t3, 13.0)
u_ref = add(P1, t1, 12.0)[0]
TOPK = (q_pad[1] - WEB_TOP_AT_SADDLE) / (q_pad[0] - u_ref)


def top_x(u):
    return q_pad[1] + TOPK * (u - q_pad[0])


def lift_f(z):
    return lambda a, b: V(a, b, z)


def build() -> Part.Shape:
    # ---- saddle + web as one prism along the pipe axis (frame F) ----------
    delta = math.atan2(FLAT_OUT, FLAT_DIST)
    h6 = (WC + WEB_HALF - ROOT1[1]) / r1[1]
    pts = [
        polar(R_PIPE, TH3 - delta),            # bore, row-3 end
        add(P3, t3, -FLAT_OUT),                # row-3 flat, outer corner
        ROOT3,                                 # web leaves the row-3 flat
        (66.0, WC - WEB_HALF),                 # ... and straightens into the neck
        (125.0, WC - WEB_HALF),
        (125.0, WC + WEB_HALF),
        add(ROOT1, r1, h6),                    # shoulder above the row-1 flat
        ROOT1,
        add(P1, t1, -FLAT_OUT),                # row-1 flat, outer corner
        polar(R_PIPE, TH1 + delta),            # bore, row-1 end
    ]
    radii = [0, 3.0, ROOT_FILLET, 15.0, 0, 0, 5.0, ROOT_FILLET, 3.0, 0]
    face = outline(pts, radii, lift_f(-SADDLE_HALF),
                   closing_arc_mid=polar(R_PIPE, (TH1 + TH3) / 2))
    body = face.extrude(V(0, 0, 2 * SADDLE_HALF))
    body.transformShape(M_F)

    # the web stops inside the pad's stem
    stop = Part.makeBox(400, 400, 200, V(-200, -200, -200 + PAD_T + WEB_FILLET + 0.1))
    stop.transformShape(M_B)
    body = body.cut(stop)

    # ---- pad for B with its stem and fillets, extruded along c (frame B) ---
    c_lo = -60.0
    n_stem = PAD_T + 5.0
    tpts = [(-PAD_HALF - 4, -2), (PAD_HALF + 4, -2), (PAD_HALF + 4, PAD_T),
            (WEB_HALF, PAD_T), (WEB_HALF, n_stem), (-WEB_HALF, n_stem),
            (-WEB_HALF, PAD_T), (-PAD_HALF - 4, PAD_T)]
    trad = [0, 0, 0, WEB_FILLET, 0, 0, WEB_FILLET, 0]
    tee = outline(tpts, trad, lambda w, n: V(c_lo, w, n)).extrude(V(PAD_HALF + 10 - c_lo, 0, 0))

    # side view (c, n): seat, then the relieved back running down to the bed
    al = math.radians(RELIEF_SLOPE - SEAT_OVERHANG)
    assert 0.03 < al < 0.12
    p_arc_end = (SEAT_END - RELIEF_R * math.sin(al), RELIEF_R * (1 - math.cos(al)))
    p_arc_mid = (SEAT_END - RELIEF_R * math.sin(al / 2), RELIEF_R * (1 - math.cos(al / 2)))
    p_low = (c_lo, p_arc_end[1] + math.tan(al) * (p_arc_end[0] - c_lo))
    lift_side = lambda c, n: V(c, -40.0, n)
    e = [Part.LineSegment(lift_side(SEAT_END, 0), lift_side(PAD_HALF, 0)),
         Part.LineSegment(lift_side(PAD_HALF, 0), lift_side(PAD_HALF, 30)),
         Part.LineSegment(lift_side(PAD_HALF, 30), lift_side(c_lo, 30)),
         Part.LineSegment(lift_side(c_lo, 30), lift_side(*p_low)),
         Part.LineSegment(lift_side(*p_low), lift_side(*p_arc_end)),
         Part.Arc(lift_side(*p_arc_end), lift_side(*p_arc_mid), lift_side(SEAT_END, 0))]
    side = Part.Face(Part.Wire([x.toShape() for x in e])).extrude(V(0, 80, 0))

    # plan view (c, w): top corners rounded about the bolts
    ppts = [(c_lo - 5, -PAD_HALF), (PAD_HALF, -PAD_HALF), (PAD_HALF, PAD_HALF), (c_lo - 5, PAD_HALF)]
    plan = outline(ppts, [0, PAD_CORNER, PAD_CORNER, 0], lambda c, w: V(c, w, -5.0)).extrude(V(0, 0, 40))

    pad = tee.common(side).common(plan)
    pad.transformShape(M_B)
    # the build plate: everything stops at the saddle's lower end
    below = Part.makeBox(600, 600, 200, V(-300, -300, -SADDLE_HALF - 200))
    below.transformShape(M_F)
    pad = pad.cut(below)

    part = body.fuse(pad)

    # ---- gusset line: remove web and stem above the sloped top plane -------
    u_a, u_b = -100.0, 300.0
    lift_top = lambda u, x: V(u, -300.0, x)
    tp = [(u_a, top_x(u_a)), (u_b, top_x(u_b)), (u_b, 300.0), (u_a, 300.0)]
    above = outline(tp, [0, 0, 0, 0], lift_top).extrude(V(0, 600, 0))
    apex = polar(FLAT_DIST / math.cos((TH1 - TH3) / 2), (TH1 + TH3) / 2)
    zpts = [apex, add(P3, t3, -90.0), (250.0, add(P3, t3, -90.0)[1]),
            (250.0, 200.0), add(P1, t1, -90.0)]
    zone = outline(zpts, [0] * 5, lift_f(-100.0)).extrude(V(0, 0, 200))
    cutter = above.common(zone)
    cutter.transformShape(M_F)
    front = Part.makeBox(600, 600, 300, V(-300, -300, PAD_T))   # in front of the pad
    front.transformShape(M_B)
    part = part.cut(cutter.common(front))

    # ---- bolt holes ---------------------------------------------------------
    for b in a_bolts:
        p, ax = V(*b["position"]), V(*b["axis"])
        part = part.cut(Part.makeCylinder(D_A / 2, 14.0, p - ax * 4.0, ax))
    for b in b_bolts:
        p, ax = V(*b["position"]), V(*b["axis"])
        part = part.cut(Part.makeCylinder(D_B / 2, 11.0, p - ax * 3.0, ax))
    return part


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build().removeSplitter()
    assert len(shape.Solids) == 1, "expected one solid, got %d" % len(shape.Solids)
    shape = shape.Solids[0]
    assert shape.isValid() and shape.isClosed()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume %.0f mm3, %d faces" % (shape.Volume, len(shape.Faces)))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
