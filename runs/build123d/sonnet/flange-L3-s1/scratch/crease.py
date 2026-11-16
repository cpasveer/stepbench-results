import sys, math; sys.path.insert(0,'.')
import submission as S
from build123d import *
p=S.build()
faces=p.faces()
# map edges to faces
from collections import defaultdict
emap=defaultdict(list)
for i,f in enumerate(faces):
    for e in f.edges():
        emap[(round(e.center().X,3),round(e.center().Y,3),round(e.center().Z,3),round(e.length,3))].append(i)
cnt=0
for k,v in emap.items():
    if len(v)==2:
        a,b=faces[v[0]],faces[v[1]]
        c=Vector(k[0],k[1],k[2])
        try:
            na=a.normal_at(c); nb=b.normal_at(c)
        except Exception: continue
        ang=math.degrees(math.acos(max(-1,min(1,na.dot(nb)))))
        if 0.01<ang<3.0:
            cnt+=1; print('crease',round(ang,2),v,a.geom_type,b.geom_type,k)
print('creases',cnt,'faces',len(faces))
