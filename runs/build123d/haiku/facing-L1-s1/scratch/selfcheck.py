import sys, json, math, pathlib
sys.path.insert(0, str(pathlib.Path("/home/user/stepbench-runs/tasks/facing-L1-s1")))
from build123d import import_step, Pos, Cylinder, Box
from submission import build
p = build()
print("valid", p.is_valid, "solids", len(p.solids()), "volume", round(p.volume,1))
bb = p.bounding_box(); print("bbox", bb.min, bb.max)
small = [e for e in p.edges() if e.length < 0.1]; print("short edges", len(small))
faces = p.faces(); print("faces", len(faces), "min face area", round(min(f.area for f in faces),2))
for f in faces:
    if f.area < 0.25: print("tiny face", f.area)
for name in ["fixed_A.step","fixed_B.step"]:
    q = import_step(str(pathlib.Path("/home/user/stepbench-runs/tasks/facing-L1-s1")/name))
    b=q.bounding_box(); print(name, b.min, b.max)
    ov = p.intersect(q).volume if p.intersect(q) else 0
    print(" overlap volume", round(ov,4))
