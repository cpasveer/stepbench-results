from build123d import *
for f in ["fixed_A.step","fixed_B.step"]:
    s=import_step(f); bb=s.bounding_box(); print(f, len(s.solids()), bb.min, bb.max, s.volume)
