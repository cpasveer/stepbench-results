"""pipe-L3-s1 connector: FreeCAD script, writes output.step.

Design (printed standing on the pipe-axis direction, no supports):
  * two rails that lie along the pipe, one per bolt row, each with a flat
    outer face square to its bolts (cap-screw heads bear on a flat),
  * a 4 mm plate on the flat mount B,
  * a rectangular beam between them, running between the bolt columns of B
    and widening to the full gap between the rails at the pipe,
  * a foot under the plate's lowest edge so the inclined plate starts on
    the print bed.
Everything except the plate is a prism along the pipe axis, which is the
print direction (bed = the rail ends); the plate leans 50 deg from the bed.
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
RES = POSE["resolved"]
BOLTS = RES["bolts"]
V = App.Vector

# ---------------------------------------------------------------- parameters
R_PIPE = TASK["fixed_parts"][0]["interface"]["params"]["od"] / 2.0
PLATE_HALF = 16.5     # plate on B: 34 x 34 (B face is 38 x 38)
PLATE_T = 3.5         # grip of the M4 bolts
RAIL_G = 4.0          # grip of the M5 bolts (rail thickness on the bolt axis)
RAIL_W = 5.25          # rail half width
RAIL_END = 5.5        # rail length beyond the bolt centres
BEAM_W = 8.3          # beam half width between B's bolt columns
BEAM_H = 24.0         # beam height above the bed
Q_KINK = 38.0         # where the beam starts to widen (past B's tool columns)
INSET = 0.0           # beam edges meet the rail-to-rail chord this far in
FLARE_H = 0.0         # height of the widening part (0: same as the beam)
BEAM_W2 = 0.0         # beam half width at the kink (0: same as BEAM_W)
ROOT_H = 8.0          # beam height at Q_ROOT, top sloping down from the kink (0: flat)
Q_ROOT = 72.0
D_A = 5.5             # M5 clearance hole
D_B = 4.5             # M4 clearance hole
OUT_NAME = "output.step"
# scratch studies may override parameters: PARAMS="BEAM_H=20 OUT_NAME=scratch/x.step"
for _kv in os.environ.get("PARAMS", "").split():
    _k, _v = _kv.split("=")
    globals()[_k] = _v if _k == "OUT_NAME" else float(_v)


def build() -> Part.Shape:
    # ---- frames -------------------------------------------------------
    pa = POSE["placements"]["A"]
    pb = POSE["placements"]["B"]
    rot_a = pa["rotation"]
    D = V(rot_a[0][0], rot_a[1][0], rot_a[2][0])      # pipe axis = print axis
    D.normalize()
    c_pipe_w = V(*pa["origin"])
    O = V(*pb["origin"])                               # centre of B's face
    N = V(*RES["mounting_faces"][0]["normal"])
    N.normalize()
    Q = N - D * N.dot(D)
    Q.normalize()
    U = Q.cross(D)
    cn, sn = N.dot(Q), N.dot(D)                        # N = cn*Q + sn*D

    def W(x, y, z):
        """local (u, q, d) -> world"""
        return O + U * x + Q * y + D * z

    def L(p):
        r = p - O
        return (r.dot(U), r.dot(Q), r.dot(D))

    def prism(pts_xy, z0, z1):
        """polygon in the local u-q plane, extruded along the pipe axis"""
        pts = [W(x, y, z0) for x, y in pts_xy]
        face = Part.Face(Part.makePolygon(pts + [pts[0]]))
        return face.extrude(D * (z1 - z0))

    a_bolts = [b for b in BOLTS if b["id"].startswith("A")]
    b_bolts = [b for b in BOLTS if b["id"].startswith("B")]
    cx, cy, _ = L(c_pipe_w)
    a_loc = [L(V(*b["position"])) for b in a_bolts]
    z_lo = min(p[2] for p in a_loc) - RAIL_END         # top of the print
    z_bed = max(p[2] for p in a_loc) + RAIL_END        # print bed

    # ---- rails on the pipe -------------------------------------------
    rows = {}
    for p in a_loc:
        ang = math.atan2(p[1] - cy, p[0] - cx)
        rows.setdefault(round(math.degrees(ang), 1), ang)
    angs = sorted(rows.values())
    assert len(angs) == 2
    rdir = [(math.cos(a), math.sin(a)) for a in angs]
    rails, corner, inner = [], [], []
    ro = R_PIPE + RAIL_G
    for i in (0, 1):
        rx, ry = rdir[i]
        ox, oy = rdir[1 - i]
        tx, ty = -ry, rx
        if tx * ox + ty * oy < 0:                      # t points to the other row
            tx, ty = -tx, -ty
        pt = lambda rho, s: (cx + rho * rx + s * tx, cy + rho * ry + s * ty)
        rails.append(prism([pt(25.0, -RAIL_W), pt(ro, -RAIL_W),
                            pt(ro, RAIL_W), pt(25.0, RAIL_W)], z_lo, z_bed))
        corner.append(pt(ro, RAIL_W))
        inner.append(pt(25.0, 0.0))
    right, left = (0, 1) if corner[0][0] > corner[1][0] else (1, 0)

    # ---- plate on B with the foot under its lowest edge ---------------
    hp, tp = PLATE_HALF, PLATE_T
    sp = lambda s, m: (-s * sn + m * cn, s * cn + m * sn)   # (q, d)
    a_, b_, c_, d_ = sp(hp, 0), sp(-hp, 0), sp(-hp, tp), sp(hp, tp)
    prof = [a_, b_, c_, d_, (d_[0], z_bed), (a_[0], z_bed)]
    pts = [W(-hp, y, z) for y, z in prof]
    plate = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(U * (2 * hp))
    # taper the foot down to the beam width (51 deg from the bed)
    k = (z_bed + 1.0 - d_[1]) / (z_bed - d_[1])
    for sgn in (1, -1):
        tri = [(hp, d_[1]), (hp + (BEAM_W - hp) * k, z_bed + 1.0),
               (hp + 2.0, z_bed + 1.0), (hp + 2.0, d_[1])]
        tp_ = [W(sgn * x, a_[0] - 1.0, z) for x, z in tri]
        cutter = Part.Face(Part.makePolygon(tp_ + [tp_[0]])).extrude(
            Q * (d_[0] - a_[0] + 2.0))
        plate = plate.cut(cutter)

    # ---- beam ---------------------------------------------------------
    q0 = 0.5 * (a_[0] + d_[0])
    cr, cl = corner[right], corner[left]
    clen = math.hypot(cl[0] - cr[0], cl[1] - cr[1])
    ch = ((cl[0] - cr[0]) / clen, (cl[1] - cr[1]) / clen)   # chord, right to left
    w2 = BEAM_W2 if BEAM_W2 > 0 else BEAM_W
    poly = [(-BEAM_W, q0), (BEAM_W, q0), (w2, Q_KINK)]
    if INSET > 0:
        poly.append((cr[0] + INSET * ch[0], cr[1] + INSET * ch[1]))
    poly += [cr, inner[right], inner[left], cl]
    if INSET > 0:
        poly.append((cl[0] - INSET * ch[0], cl[1] - INSET * ch[1]))
    poly.append((-w2, Q_KINK))
    fl_h = FLARE_H if FLARE_H > 0 else BEAM_H
    beam = prism(poly, z_bed - fl_h, z_bed)
    if fl_h < BEAM_H:
        beam = beam.fuse(prism([(-BEAM_W, q0), (BEAM_W, q0), (w2, Q_KINK),
                                (-w2, Q_KINK)], z_bed - BEAM_H, z_bed))
    if ROOT_H > 0:
        zt = z_bed - BEAM_H
        sl = (BEAM_H - ROOT_H) / (Q_ROOT - Q_KINK)
        side = [(Q_KINK, zt), (Q_KINK + 80.0, zt + 80.0 * sl),
                (Q_KINK + 80.0, zt - 20.0), (Q_KINK, zt - 20.0)]
        sp_ = [W(-60.0, y, z) for y, z in side]
        beam = beam.cut(Part.Face(Part.makePolygon(sp_ + [sp_[0]])).extrude(U * 120.0))
    # keep only what is in front of the plate's mid plane
    big = 400.0
    keep = Part.makeBox(big, big, big, O + N * (tp / 2.0) - U * (big / 2)
                        - N.cross(U) * (big / 2), N)
    # makeBox: z axis along N, base corner shifted; rotate about N is free
    keep = Part.makeBox(big, big, big)
    m = App.Matrix(U.x, N.cross(U).x, N.x, 0,
                   U.y, N.cross(U).y, N.y, 0,
                   U.z, N.cross(U).z, N.z, 0,
                   0, 0, 0, 1)
    keep.translate(V(-big / 2, -big / 2, tp / 2.0))
    keep = keep.transformGeometry(m)
    keep.translate(O)
    beam = beam.common(keep)

    body = plate.fuse(beam)
    for r in rails:
        body = body.fuse(r)

    # ---- pipe seat and holes -------------------------------------------
    pipe = Part.makeCylinder(R_PIPE, 400.0, c_pipe_w - D * 200.0, D)
    body = body.cut(pipe)
    for b in a_bolts:
        p, ax = V(*b["position"]), V(*b["axis"])
        body = body.cut(Part.makeCylinder(D_A / 2, 20.0, p - ax * 6.0, ax))
    for b in b_bolts:
        p, ax = V(*b["position"]), V(*b["axis"])
        body = body.cut(Part.makeCylinder(D_B / 2, tp + 2.0, p - ax, ax))
    return body.removeSplitter()


OUT = os.path.join(HERE, OUT_NAME)
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    assert shape.isValid() and len(shape.Solids) == 1, "not one valid solid"
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "faces", len(shape.Faces))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
