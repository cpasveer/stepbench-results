import sys
sys.path.insert(0, "/home/user/stepbench-runs/tasks/pipe-L1-s1")
from build123d import import_step, Vector
import submission as S
p = import_step("/home/user/stepbench-runs/tasks/pipe-L1-s1/output.step")
print("solids", len(p.solids()), "valid", p.is_valid, "vol", round(p.volume,1))
for f in p.faces():
    if f.area < 2.0:
        print("small face area", round(f.area,4), "center", f.center().round(2) if hasattr(f.center(),'round') else f.center(), f.geom_type)
A = import_step("/home/user/stepbench-runs/tasks/pipe-L1-s1/fixed_A.step")
B = import_step("/home/user/stepbench-runs/tasks/pipe-L1-s1/fixed_B.step")
for n, F in (("A", A), ("B", B)):
    i = p.intersect(F)
    print("overlap", n, None if i is None else round(i.volume, 5))
