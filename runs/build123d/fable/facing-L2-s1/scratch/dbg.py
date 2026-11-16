import sys, math
from build123d import *
p=import_step("output.step").solids()[0]
pt=Vector(-85.73,121.13,-32.01)
fs=sorted(p.faces(), key=lambda f:f.distance_to(pt))[:3]
for f in fs:
    print(f.geom_type, f.distance_to(pt), f.normal_at(f.center()))
f=fs[0]; n=f.normal_at(f.center())
for h in p.find_intersection_points(Axis(pt-n*1e-3,n*-1)): print(h, (h[0]-pt).length)
