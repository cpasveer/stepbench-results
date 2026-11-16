from build123d import *
for f in ["fixed_A.step","fixed_B.step"]:
    s=import_step(f)
    bb=s.bounding_box()
    print(f, bb.min, bb.max, len(s.solids()), [round(x.volume) for x in s.solids()], len(s.faces()))
