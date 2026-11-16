import sys; sys.path.insert(0,".")
import submission as S
from build123d import *
for n in ("0001","0002","0003"):
    p=import_step(f"attempts/{n}/output.step")
    q=Location((S.ORG.X,S.ORG.Y,0),(0,0,S.ANG)).inverse()*p
    print(n)
    for i,f in enumerate(q.faces()):
        if f.area>1000 or i in (1,15):
            c=f.center()
            try: nn=tuple(round(x,2) for x in f.normal_at(c))
            except: nn=None
            print("  ",i,f.geom_type,round(f.area),nn)
