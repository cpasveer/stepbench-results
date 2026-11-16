from build123d import *
for f in ['fixed_A.step','fixed_B.step']:
    s=import_step(f)
    print(f, s.bounding_box(), len(s.solids()), [ (fc.geom_type, round(fc.area,1)) for fc in s.faces()][:12])
