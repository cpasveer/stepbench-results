import json,numpy as np
from build123d import *
T=json.load(open('task.json'));B=T['poses'][0]['resolved']['bolts']
s=import_step('output.step')
for b in B:
    p=np.array(b['position']);a=np.array(b['axis'])
    g=8.0 if b['size']=='M5' else 7.0
    r=b['head_diameter']/2+0.5; L=b['head_height']+1
    c=Cylinder(r,L).locate(Location(Plane(origin=Vector(*(p+a*(g+0.01))),z_dir=Vector(*a)))*Pos(0,0,L/2))
    i=s&c; print(b['id'], round(i.volume,3) if i else 0)
