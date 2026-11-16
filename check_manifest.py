"""Check one task folder against the frozen v1 manifest (MANIFEST.json at the repository root).

    python3 check_manifest.py tasks/facing-L2-s0     -> "manifest ok", or the files that differ
    python3 check_manifest.py tasks-freecad/facing-L2-s1   (the FreeCAD variant: manifest key "tasks_freecad")
    python3 check_manifest.py tasks-rhino/facing-L2-s1     (the Rhino variant: manifest key "tasks_rhino")
"""
import hashlib
import json
import os
import sys

root = os.path.dirname(os.path.abspath(__file__))
folder = os.path.abspath(sys.argv[1])
task = os.path.basename(folder)
manifest = json.load(open(os.path.join(root, "MANIFEST.json")))
# a folder under tasks-freecad/ is checked against the FreeCAD variant's entries
key = {"tasks-freecad": "tasks_freecad", "tasks-rhino": "tasks_rhino"}.get(os.path.basename(os.path.dirname(folder)), "tasks")
want = manifest.get(key, {}).get(task)
if want is None:
    sys.exit(f"{task} is not in the frozen manifest ({key})")
bad = []
for name, sha in want.items():
    p = os.path.join(folder, name)
    got = hashlib.sha256(open(p, "rb").read()).hexdigest() if os.path.exists(p) else None
    if got != sha:
        bad.append(name)
if bad:
    sys.exit("manifest MISMATCH: " + ", ".join(bad))
print("manifest ok")
