from build123d import *
for f in ["fixed_A.step","fixed_B.step"]:
    s=import_step(f); bb=s.bounding_box()
    print(f, bb.min, bb.max, s.volume, len(s.solids()), len(s.faces()))
    for fc in s.faces(): print("  ", fc.geom_type, round(fc.area,1), fc.center())
