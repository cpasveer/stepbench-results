import json
from build123d import *
T=json.load(open('task.json'))['poses'][0]['resolved']['bolts']
s=import_step('output.step')
for b in T:
    x,y,z=b['position']
    if b['id'][0]=='B':
        cyl=Pos(x,y,-30.0-70)*Cylinder(4.75,70,align=(Align.CENTER,Align.CENTER,Align.MIN))
        i=s&cyl
        print(b['id'], i.volume, i.bounding_box() if i.volume>1e-6 else '')
b=T[4]; x,y,z=b['position']
cyl=Pos(x,y,-100)*Cylinder(4.75,70,align=(Align.CENTER,Align.CENTER,Align.MIN))
for sol in (s&cyl).solids():
    print(sol.volume, sol.bounding_box())
import math
print(math.hypot(x+124.18164,y-135.11207))
for f in (s&cyl).faces(): print(f.geom_type, f.area, f.center())
