import math, sys
from build123d import *
sys.path.insert(0,'.')
import submission as S
p=import_step("output.step").solids()[0]
# pocket for B3: local region
bx,by,bz=S.BOLTS[6]["position"]
near=[f for f in p.faces() if (f.center()-Vector(bx,by,f.center().Z)).length<12 and f.center().Z>-50]
print(len(near))
# edges
for f in near:
    if f.geom_type!=GeomType.PLANE or abs(f.normal_at(f.center()).Z)>0.99: continue
    n=f.normal_at(f.center())
    print("face",round(f.area,1),"n",[round(v,3) for v in n], "lean from vertical", round(90-math.degrees(math.acos(abs(n.Z))),1))
    for e in f.edges():
        worst=0
        for t in [i/20 for i in range(21)]:
            tg=e.tangent_at(t); worst=max(worst, math.degrees(math.acos(min(1,abs(tg.Z)))))
        print("   edge",e.geom_type,round(e.length,2),"max angle from vertical",round(worst,1))
# thickness sampling
import random
random.seed(1)
mn=9
cnt=0
for f in near+[f for f in p.faces() if f.geom_type==GeomType.CONE]+[f for f in p.faces() if f.geom_type==GeomType.CYLINDER and f.area>1500 and f.area<2500]:
    for i in range(4000 if f.area>100 else 1500):
        u,v=random.random(),random.random()
        try:
            pt=f.position_at(u,v); n=f.normal_at(pt)
        except Exception: continue
        if not f.is_inside(pt, 1e-4): continue
        if (pt-Vector(bx,by,pt.Z)).length>14: continue
        ax=Axis(pt-n*1e-3, n*-1)
        hits=[(h[0]-pt).length for h in p.find_intersection_points(ax) if (h[0]-pt).dot(n*-1)>0]
        hits=[h for h in hits if h>2e-3]
        if hits:
            cnt+=1
            t=min(hits)
            if t<mn or t<0.8: mn=min(t,mn); print("thin",round(t,3),f.geom_type,[round(c,2) for c in pt])
print("samples",cnt,"min",mn)
