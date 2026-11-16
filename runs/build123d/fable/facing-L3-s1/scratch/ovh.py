import sys,math; sys.path.insert(0,".")
import submission as S
from build123d import *
p=import_step(sys.argv[1])
inv=Location((S.ORG.X,S.ORG.Y,0),(0,0,S.ANG)).inverse()
q=inv*p
for i,f in enumerate(q.faces()):
    c=f.center(); 
    try: n=f.normal_at(c)
    except Exception as e: n=None
    if n is None: continue
    if n.Y< -0.3:
        print(i,f.geom_type,round(f.area,1),"c",tuple(round(x,2) for x in c),"n",tuple(round(x,3) for x in n), "ang",round(90-math.degrees(math.asin(-n.Y)),1))
