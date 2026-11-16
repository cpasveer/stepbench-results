"""Your answer to this task.

Write ONE solid to output.step (next to this file) that bolts to both fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely, or model
the part in any CAD program and save it as output.step instead.

    python submission.py                 # writes output.step
    jointbench verify . --by <your name> --tool <the CAD tool you used>

(the brief, README.md, says how to run them.)
"""
import json
import pathlib

from build123d import Box, Part, export_step

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]  # [{"part", "origin", "normal"}]
BOLTS = POSE["bolts"]  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]


def build() -> Part:
    # Placeholder: a 10 mm cube. It will fail. Replace it with your connector.
    return Box(10, 10, 10)


if __name__ == "__main__":
    export_step(build(), str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
