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


T_A = 6.0        # flange thickness on A (M4 grip)
T_B = 6.0        # flange thickness on B (M5 grip)
SETBACK = 1.0    # seat stays this far inside each mounting face
COL_R = 21.0     # central column outer radius
BORE_R = 14.0    # central through bore (weight relief / cable pass)
FILLET = 6.0     # column-to-flange root fillet
CHAMFER = 1.0    # free outer edges of the flanges


def build() -> Part.Shape:
    parts = TASK["fixed_parts"]
    place = TASK["poses"][0]["placements"]
    zA = FACES[0]["origin"][2]
    zB = FACES[1]["origin"][2]
    cB = V(*place["B"]["origin"])

    # --- flange on A: rounded square, 1 mm inside the 73 mm face -----------
    half = parts[0]["interface"]["face_size"][0] / 2.0 - SETBACK
    pitch = parts[0]["interface"]["params"]["pitch"] / 2.0
    rc = half - pitch                      # corner radius concentric with bolt
    plate = Part.makeBox(2 * half, 2 * half, T_A, V(-half, -half, 0))
    corner_edges = [e for e in plate.Edges
                    if abs(e.Vertexes[0].Point.z - e.Vertexes[-1].Point.z) > T_A / 2]
    plate = plate.makeFillet(rc, corner_edges)
    top_edges = [e for e in plate.Edges
                 if all(abs(v.Point.z - T_A) < 1e-6 for v in e.Vertexes)]
    plate = plate.makeChamfer(CHAMFER, top_edges)
    R = place["A"]["rotation"]
    o = place["A"]["origin"]
    import math
    ang = math.degrees(math.atan2(R[1][0], R[0][0]))   # A is only turned about Z
    plate.Placement = App.Placement(V(*o), App.Rotation(V(0, 0, 1), ang))

    # --- flange on B: disc, 1 mm inside the round face ---------------------
    rB = parts[1]["interface"]["face_size"][0] / 2.0 - SETBACK
    disc = Part.makeCylinder(rB, T_B, V(cB.x, cB.y, zB - T_B))
    low = [e for e in disc.Edges
           if all(abs(v.Point.z - (zB - T_B)) < 1e-6 for v in e.Vertexes)]
    disc = disc.makeChamfer(CHAMFER, low)

    # --- column on B's axis, filleted into both flanges ---------------------
    col = Part.makeCylinder(COL_R, (zB - T_B) - (zA + T_A), V(cB.x, cB.y, zA + T_A))
    body = plate.fuse(col).fuse(disc).removeSplitter()
    roots = [e for e in body.Edges
             if isinstance(e.Curve, Part.Circle) and abs(e.Curve.Radius - COL_R) < 1e-6
             and (abs(e.Vertexes[0].Point.z - (zA + T_A)) < 1e-6
                  or abs(e.Vertexes[0].Point.z - (zB - T_B)) < 1e-6)]
    assert len(roots) == 2, len(roots)
    body = body.makeFillet(FILLET, roots)

    # --- central bore and bolt holes ---------------------------------------
    body = body.cut(Part.makeCylinder(BORE_R, zB - zA + 2, V(cB.x, cB.y, zA - 1)))
    for b in BOLTS:
        lo, hi = b["clearance_diameter_range"]
        d = round((lo + hi) / 2.0 - 0.05, 1)       # 4.5 for M4, 5.5 for M5
        p, a = V(*b["position"]), V(*b["axis"])
        t = T_A if a.z > 0 else T_B
        body = body.cut(Part.makeCylinder(d / 2.0, t + 2, p - a, a))
    return body


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
