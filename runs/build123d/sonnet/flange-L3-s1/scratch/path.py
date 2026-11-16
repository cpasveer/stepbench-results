import sys, math, itertools; sys.path.insert(0,'.')
from submission import frame, BOLTS, Vector
oA,nA,tA=frame("A"); oB,nB,tB=frame("B")
def pt2(v): return (v.X,v.Y)
def seg_dist(p,a,b):
    ax,ay=a;bx,by=b;px,py=p
    dx,dy=bx-ax,by-ay
    t=max(0,min(1,((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy)))
    return math.hypot(px-(ax+t*dx),py-(ay+t*dy))
# bolts relevant (plan positions), required clearance from path centreline = req + halfwidth
rel=[]
for b in BOLTS:
    z=b["position"][2]
    rad=4.75 if b["size"]=="M5" else 5.5
    rel.append((b["id"],(b["position"][0],b["position"][1]),z,rad,Vector(*b["axis"])))
best=None
for Pn in range(20,50,2):
  for Pu in [x*0.5 for x in range(-16,17)]:
    P=oA+nA*Pn+tA*Pu
    pa,pb,p=pt2(oA),pt2(oB),pt2(P)
    worst=1e9
    for bid,pos,z,rad,ax in rel:
        # only wall z-range [-65,26]
        if z>26+rad or z< -65-rad: continue
        # bolt axis segment in plan: from pos along ax to length 40 (head+tool)
        L = 5+30+(6 if bid[0]=="B" else 5)
        pts=[(pos[0]+ax.X*s,pos[1]+ax.Y*s) for s in [i*2 for i in range(0,int(L/2)+1)]]
        d=min(min(seg_dist(q,pa,p),seg_dist(q,p,pb)) for q in pts)
        worst=min(worst,d-rad)
    if best is None or worst>best[0]: best=(worst,Pn,Pu)
print(best)
