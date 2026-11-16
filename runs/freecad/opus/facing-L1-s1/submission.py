"""Your answer to this task, as a FreeCAD script.

Write ONE solid to output.step (next to this file) that bolts to both fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely, or model
the part in FreeCAD any other way and save it as output.step instead.

    freecadcmd submission.py             # writes output.step
    jointbench verify . --by <your name> --tool freecad

(the brief, README.md, says how to run them.) Run both from this folder.

FreeCAD's Part API, all in world coordinates (millimetres):
Part.makeBox(l, w, h, App.Vector(base), App.Vector(dir)),
Part.makeCylinder(r, h, App.Vector(base), App.Vector(dir)), a.fuse(b),
a.cut(b), shape.removeSplitter(), shape.transformShape(App.Matrix),
App.Placement / App.Rotation / App.Vector. The fixed parts load with
Part.read("fixed_A.step"). No document or GUI is needed.
"""
import json
import os
import sys
import traceback

import FreeCAD as App
import Part

# freecadcmd does not always set __file__; the task folder is this file's
# folder, else the folder you run it from.
try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]  # [{"part", "origin", "normal"}]
BOLTS = POSE["bolts"]  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]
V = App.Vector


import math

T = 8.0            # flange thickness (= bolt grip)
COL_R = 19.0       # column outer radius
BORE_R = 14.0      # lightening bore through column and flanges
FILLET = 3.0       # column-to-flange fillet
EDGE = 1.5         # free outer edge round


def build() -> Part.Shape:
    fa, fb = FACES
    oa, ob = V(*fa["origin"]), V(*fb["origin"])
    za, zb = oa.z, ob.z
    ra = TASK["poses"][0]["placements"]["A"]["rotation"]
    ang_a = math.degrees(math.atan2(ra[1][0], ra[0][0]))

    # Flange on A: 70 x 70 rounded square (1.5 mm inside the 73 mm face)
    sq, cr = 70.0, 6.0
    core = Part.makeBox(sq - 2 * cr, sq, T, V(-(sq / 2 - cr), -sq / 2, 0))
    core = core.fuse(Part.makeBox(sq, sq - 2 * cr, T, V(-sq / 2, -(sq / 2 - cr), 0)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            core = core.fuse(Part.makeCylinder(cr, T, V(sx * (sq / 2 - cr), sy * (sq / 2 - cr), 0)))
    flange_a = core.removeSplitter()
    flange_a.rotate(V(0, 0, 0), V(0, 0, 1), ang_a)
    flange_a.translate(V(oa.x, oa.y, za))

    # Flange on B: 82 mm disc (0.25 mm inside the 84.5 mm face)
    flange_b = Part.makeCylinder(41.0, T, V(ob.x, ob.y, zb - T))

    # Central column between the two flanges, centred between both mounts
    cx, cy = (oa.x + ob.x) / 2, (oa.y + ob.y) / 2
    col = Part.makeCylinder(COL_R, (zb - T) - (za + T), V(cx, cy, za + T))

    body = flange_a.fuse(flange_b).fuse(col).removeSplitter()

    # fillets: column/flange junctions and free outer edges
    fil = []
    for e in body.Edges:
        c = e.Curve
        z0, z1 = e.BoundBox.ZMin, e.BoundBox.ZMax
        if abs(z1 - z0) > 1e-6:
            continue
        z = z0
        if isinstance(c, Part.Circle) and abs(c.Radius - COL_R) < 1e-6 and (
                abs(z - (za + T)) < 1e-6 or abs(z - (zb - T)) < 1e-6):
            fil.append(e)
    body = body.makeFillet(FILLET, fil)
    body = body.makeFillet(EDGE, _outer_edges(body, za + T, zb - T, cx, cy))

    # lightening bore
    body = body.cut(Part.makeCylinder(BORE_R, zb - za, V(cx, cy, za)))

    # clearance holes
    for b in BOLTS:
        d = 4.5 if b["size"] == "M4" else 5.5
        p = V(*b["position"])
        a = V(*b["axis"])
        body = body.cut(Part.makeCylinder(d / 2, T + 2, p - a * 1, a))
    return body.removeSplitter()


def _outer_edges(body, ztop_a, zbot_b, cx, cy):
    out = []
    for e in body.Edges:
        bb = e.BoundBox
        if abs(bb.ZMax - bb.ZMin) > 1e-6:
            continue
        z = bb.ZMin
        if not (abs(z - ztop_a) < 1e-6 or abs(z - zbot_b) < 1e-6):
            continue
        m = e.valueAt((e.FirstParameter + e.LastParameter) / 2)
        if math.hypot(m.x - cx, m.y - cy) > 30:
            out.append(e)
    return out


# freecadcmd exits 0 even when a script fails; this makes a failure visible
# (full traceback, exit 1) and never leaves a stale output.step behind.
OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
