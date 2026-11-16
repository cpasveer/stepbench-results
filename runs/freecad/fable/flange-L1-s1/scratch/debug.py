"""Connector for flange-L1-s1: two seat plates joined by a tapered trunk.

A round plate seats on flange A and a rectangular plate on block B.  A
solid, rounded, tapered trunk joins them inside the bolt patterns, so every
socket head stays on an open plate face with its hex key free to swing
outwards.  Root fillets blend the trunk into both plates.

    freecadcmd submission.py     # writes output.step
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

try:
    HERE = os.getcwd()
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
IFACE = {p["name"]: p["interface"] for p in TASK["fixed_parts"]}
V = App.Vector
Z = V(0, 0, 1)

# ---- design parameters (mm) ------------------------------------------------
T_A = 10.0          # plate thickness on A (M5 grip 3.75-25)
T_B = 10.0          # plate thickness on B (M6 grip 4.5-25)
SETBACK = 1.25      # seat stays this far inside the mounting-face outline
HOLE = {"M5": 5.5, "M6": 6.6}   # ISO 273 medium clearance
A_U = (-18.0, 22.0)   # trunk footprint on plate A, across the face
A_Z = (-9.0, 23.5)    # trunk footprint on plate A, world z
B_U = (-20.0, 20.0)   # trunk footprint on plate B, across the face
B_Z = (-41.0, 10.0)   # trunk height on plate B at its centre line, world z
R_TRUNK = 5.0       # rounding of the trunk's four long edges
R_ROOT = 3.0        # root fillet where the trunk meets the plates


def frame(name):
    f = FACES[name]
    o, n = V(*f["origin"]), V(*f["normal"])
    n.normalize()
    u = V(n.y, -n.x, 0)          # horizontal, in the face
    return o, n, u


oA, nA, uA = frame("A")
oB, nB, uB = frame("B")
uB = uB * -1                      # so +uA and +uB are the same side


def plane_z(p1, p2, p3):
    """z(x, y) of the plane through three points."""
    nrm = (p2 - p1).cross(p3 - p1)
    return lambda x, y: p1.z - (nrm.x * (x - p1.x) + nrm.y * (y - p1.y)) / nrm.z


def trunk():
    # plan corners on the two plates' free faces, then run into the plates
    a = [oA + nA * T_A + uA * s for s in A_U]
    b = [oB + nB * T_B + uB * s for s in B_U]
    top = plane_z(a[0] + Z * A_Z[1], a[1] + Z * A_Z[1],
                  oB + nB * T_B + Z * B_Z[1])
    bot = plane_z(a[0] + Z * A_Z[0], a[1] + Z * A_Z[0],
                  oB + nB * T_B + Z * B_Z[0])
    ends = []
    for i in (0, 1):
        d = b[i] - a[i]
        ta = (T_A / 2) / d.dot(nA)            # half-way into plate A
        tb = (T_B / 2) / (-d.dot(nB))         # half-way into plate B
        ends.append((a[i] - d * ta, b[i] + d * tb))
    (a0, b0), (a1, b1) = ends
    plan = [a0, a1, b1, b0]
    # vertical prism over the plan outline, trimmed by the two sloping planes
    solid = Part.Face(Part.makePolygon([V(p.x, p.y, -200) for p in plan + [plan[0]]])).extrude(Z * 400)
    c = (oA + oB) * 0.5
    big = [V(c.x + sx * 200, c.y + sy * 200, 0) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    for fz, dz in ((top, 300), (bot, -300)):
        pts = [V(p.x, p.y, fz(p.x, p.y)) for p in big]
        solid = solid.cut(Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(Z * dz))
    # round the four long edges (those running from plate A to plate B)
    long_edges = []
    for e in solid.Edges:
        p, q = e.Vertexes[0].Point, e.Vertexes[1].Point
        if abs((q - p).dot(nA)) > 20:
            long_edges.append(e)
    assert len(long_edges) == 4
    return solid.makeFillet(R_TRUNK, long_edges)


def plates():
    ra = IFACE["A"]["face_size"][0] / 2 - SETBACK
    pa = Part.makeCylinder(ra, T_A, oA, nA)
    w, h = (s - 2 * SETBACK for s in IFACE["B"]["face_size"])
    prof = Part.makePolygon([oB + uB * (sx * w / 2) + Z * (sz * h / 2)
                             for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))])
    pb = Part.Face(prof).extrude(nB * T_B)
    # round the block plate's four corners about the bolts
    corners = [e for e in pb.Edges
               if abs((e.Vertexes[0].Point - e.Vertexes[1].Point).dot(nB)) > T_B - 0.1]
    pb = pb.makeFillet(6.0, corners)
    return pa, pb, ra, w, h


def root_edges(shape, ra, w, h):
    """Edges where the trunk meets the free face of either plate."""
    out = []
    for e in shape.Edges:
        pts = [v.Point for v in e.Vertexes] + [e.valueAt((e.FirstParameter + e.LastParameter) / 2)]
        if all(abs((p - oA).dot(nA) - T_A) < 1e-6 for p in pts):
            m = pts[-1] - oA
            if math.hypot(m.dot(uA), m.z) < ra - 0.5:
                out.append(e)
        elif all(abs((p - oB).dot(nB) - T_B) < 1e-6 for p in pts):
            m = pts[-1] - oB
            corner = isinstance(e.Curve, Part.Circle) and abs(e.Curve.Radius - 6.0) < 1e-6
            if not corner and abs(m.dot(uB)) < w / 2 - 0.5 and abs(m.z) < h / 2 - 0.5:
                out.append(e)
    return out


def build() -> Part.Shape:
    pa, pb, ra, w, h = plates()
    tr = trunk()
    print("trunk", tr.isValid(), tr.Volume, "plates", pa.isValid(), pb.isValid(), file=sys.stderr)
    body = pa.fuse(tr).fuse(pb).removeSplitter()
    print("fused", body.isValid(), len(body.Solids), body.Volume, file=sys.stderr)
    re = root_edges(body, ra, w, h)
    print("root edges", len(re), [round(e.Length,2) for e in re], file=sys.stderr)
    reA = [e for e in re if abs((e.Vertexes[0].Point - oA).dot(nA) - T_A) < 1e-6]
    reB = [e for e in re if e not in reA and not any(e.isSame(x) for x in reA)]
    for r in (1.5, 2, 3, 4):
        for nm, es in (("A", reA), ("B", reB)):
            try:
                t = body.makeFillet(r, es)
                print(r, nm, len(es), t.isValid(), len(t.Faces), file=sys.stderr)
            except Exception as ex:
                print(r, nm, "fail", ex, file=sys.stderr)
    body = body.makeFillet(R_ROOT, re)
    print("filleted", body.isValid(), len(body.Solids), len(body.Faces), file=sys.stderr)
    for bolt in BOLTS:
        p, ax = V(*bolt["position"]), V(*bolt["axis"])
        body = body.cut(Part.makeCylinder(HOLE[bolt["size"]] / 2, 40, p - ax * 5, ax))
    return body


OUT = os.path.join(HERE, "scratch/debug.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    assert shape.isValid() and len(shape.Solids) == 1
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
