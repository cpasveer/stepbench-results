import sys; sys.path.insert(0,'.')
import json
from build123d import *
import submission as S
p=S.build()
for bid in ["A3","B3"]:
    b=[x for x in S.B if x["id"]==bid][0]
    x,y,_=b["position"]
    for r in [0]:
        zs=[z/2 for z in range(-190,-40)]
        inside=[z for z in zs if p.is_inside(Vector(x+r,y,z))]
        # compress
        rng=[];
        for z in inside:
            if rng and z-rng[-1][1]<=0.6: rng[-1][1]=z
            else: rng.append([z,z])
        print(bid,r,rng)
import math
b=[x for x in S.B if x["id"]=="A3"][0]; x,y,_=b["position"]
bb=[x for x in S.B if x["id"]=="B3"][0]
for ang in range(0,360,30):
    pts=[Vector(x+2.2*math.cos(math.radians(ang)),y+2.2*math.sin(math.radians(ang)),z) for z in range(-90,-26,2)]
    print(ang,[pt.Z for pt in pts if p.is_inside(pt)])
