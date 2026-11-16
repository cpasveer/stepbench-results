from build123d import import_step
for f in ["fixed_A.step","fixed_B.step"]:
    s=import_step(f)
    bb=s.bounding_box()
    print(f, bb.min, bb.max, s.volume, len(s.solids()))
