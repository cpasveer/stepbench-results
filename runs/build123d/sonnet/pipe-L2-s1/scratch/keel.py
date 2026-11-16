import sys; sys.path.insert(0,'.')
from submission import *
part=build(); up=np.array([1.,0,0])
m={}
for f in part.faces():
    for e in f.edges():
        c=e.center(); key=tuple(np.round([c.X,c.Y,c.Z],2))
        m.setdefault(key,[]).append((f,e))
for k,l in m.items():
    if len(l)<2: continue
    ns=[]
    for f,e in l:
        p=e.center(); n=f.normal_at(p); ns.append(np.array([n.X,n.Y,n.Z]))
    if all(n@up<-0.05 for n in ns):
        e=l[0][1]; d=np.array([e.end_point().X-e.start_point().X,e.end_point().Y-e.start_point().Y,e.end_point().Z-e.start_point().Z])
        ang=math.degrees(math.acos(abs(d@up)/np.linalg.norm(d)))
        print(np.round(k,1),"len",round(e.length,1),"angle from vertical",round(ang,1),[round(float(n@up),2) for n in ns], l[0][0].geom_type, l[1][0].geom_type)
