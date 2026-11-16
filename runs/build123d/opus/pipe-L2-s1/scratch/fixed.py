from build123d import *
for f in ['fixed_A.step','fixed_B.step']:
    s=import_step(f); 
    print(f, type(s), s.bounding_box(), s.volume, len(s.solids()))
