import sys; sys.path.insert(0,'.')
from build123d import *
import submission as s
p=s.build()
print("solids",len(p.solids()),"valid",p.is_valid,"vol",p.volume,"faces",len(p.faces()))
b=p.bounding_box(); print(b.min,b.max)
for f in ["fixed_A.step","fixed_B.step"]:
    fx=import_step(f); print(f,"overlap",(p & fx).volume if (p&fx) else 0)
print(sorted(round(e.length,3) for e in p.edges())[:8])
print(sorted(round(f.area,3) for f in p.faces())[:8])
for f in sorted(p.faces(), key=lambda f:f.area):
    bb=f.bounding_box()
    print(round(f.area,2), f.geom_type, f.center(), bb.size)
