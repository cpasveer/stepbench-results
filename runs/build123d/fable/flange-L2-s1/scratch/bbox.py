from build123d import *
for f in ["fixed_A.step","fixed_B.step"]:
    s=import_step(f); b=s.bounding_box(); print(f,b.min,b.max,s.volume, len(s.solids()))
