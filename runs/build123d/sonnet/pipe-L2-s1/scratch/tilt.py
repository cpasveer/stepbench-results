import sys; sys.path.insert(0,'.'); sys.path.insert(0,'scratch')
from submission import *
import oh
part=build()
S=oh.samples(part)
# edges
emap={}
for f in part.faces():
    for e in f.edges():
        c=e.center(); emap.setdefault(tuple(np.round([c.X,c.Y,c.Z],2)),[]).append((f,e))
edges=[]
for k,l in emap.items():
    if len(l)<2: continue
    ns=[]
    for f,e in l:
        n=f.normal_at(e.center()); ns.append(np.array([n.X,n.Y,n.Z]))
    e=l[0][1]; dv=np.array([e.end_point().X-e.start_point().X,e.end_point().Y-e.start_point().Y,e.end_point().Z-e.start_point().Z])
    if np.linalg.norm(dv)<1e-6 or e.length<5: continue
    edges.append((ns,dv/np.linalg.norm(dv),e.length))
res=[]
x=np.array([1.,0,0])
for eps in [0,4,8,12,16,20]:
  for az in range(0,360,20):
    # direction perpendicular to x
    w=np.array([0,math.cos(math.radians(az)),math.sin(math.radians(az))])
    up=x*math.cos(math.radians(eps))+w*math.sin(math.radians(eps))
    bad=0
    for n,wt,f in S:
        if abs(n@x+1)<1e-3 or abs(n@x-1)<1e-3: continue   # flat end faces (recut)
        c=-(n@up)
        if c>0.7071:
            if f.geom_type==GeomType.CYLINDER and f.area<150: continue
            bad+=wt
    kl=0
    for ns,dv,L in edges:
        if all(n@up<-0.02 for n in ns) and not all(abs(n@x+1)<1e-3 or abs(n@x-1)<1e-3 for n in ns):
            ang=math.degrees(math.acos(min(1,abs(dv@up))))
            if ang>63.4: kl+=L
    res.append((bad+kl*5,eps,az,round(bad),round(kl)))
res.sort()
for r in res[:12]: print(r)
