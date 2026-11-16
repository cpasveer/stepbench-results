import math
from build123d import import_step, Vector
p = import_step("/home/user/stepbench-runs/tasks/facing-L3-s1/output.step")
dirs = {"+Z":(0,0,1),"-Z":(0,0,-1),"+X":(1,0,0),"-X":(-1,0,0),"+Y":(0,1,0),"-Y":(0,-1,0)}
for name,d in dirs.items():
    dv = Vector(*d)
    tot = 0.0
    for f in p.faces():
        try:
            n = f.normal_at(f.center())
        except Exception:
            continue
        c = n.dot(dv)           # >0: facing build direction (up-facing), <0: downward
        if c < -0.707:          # overhang steeper than 45 deg from the build plate
            tot += f.area * (-c)
    print(name, round(tot,1))
