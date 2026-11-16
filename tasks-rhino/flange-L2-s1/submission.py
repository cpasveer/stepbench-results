#! python3
"""Your answer to this task, modelled in Rhino.

Write ONE solid to output.step (next to this file) that bolts to the fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely.

    sh run_rhino.sh                                  # runs this script inside Rhino, writes output.step
    jointbench verify . --by <your name> --tool rhino

The script builds the part in its own invisible Rhino document (millimetres,
world coordinates) and exports it as STEP. Its messages and any error go to
submission.log; on an error no output.step is left behind.
"""
import json
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = POSE.get("mounting_faces", [])  # [{"part", "origin", "normal"}]
BOLTS = POSE.get("bolts", [])  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]


def build():
    """Return a list of closed Breps forming ONE solid. Placeholder: a 10 mm cube. It will fail."""
    box = rg.Box(rg.Plane.WorldXY, rg.Interval(0, 10), rg.Interval(0, 10), rg.Interval(0, 10))
    return [box.ToBrep()]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in build():
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
