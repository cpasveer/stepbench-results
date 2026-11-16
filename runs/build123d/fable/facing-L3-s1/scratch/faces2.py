import sys; sys.path.insert(0,".")
import submission as S
from build123d import *
p=import_step("output.step")
q=Location((S.ORG.X,S.ORG.Y,0),(0,0,S.ANG)).inverse()*p
L=[]
for i,f in enumerate(q.faces()):
    c=f.center()
    try: nn=tuple(round(x,2) for x in f.normal_at(c))
    except: nn=None
    L.append((round(f.area),i,str(f.geom_type),nn))
for x in sorted(L,reverse=True)[:8]: print(x)
