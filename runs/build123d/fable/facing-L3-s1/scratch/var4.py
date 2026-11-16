import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
def design(H,U1=-24.0,U2=15.6,win=True,ub=8,ut=-15,winA=None,HD=None,wtop=5.6):
    HD = H if HD is None else HD
    oB=[]
    if H<67.7: oB.append(rect(U1-1,U2-6.8,H,70))
    if win: oB.append(window(ut+0.63+5.6,U2-6.8,6,H-wtop))
    oA=[rect(U1+6.8,ub-0.63-5.6,6,70)]
    return dict(TW=1.2,U1=U1,U2=U2,webs=[(U1+0.6,0,H,1.2,[]),(U2-0.6,0,H,1.2,[])],diags=[(ub,ut,0,HD,1.2)],openA=oA,openB=oB)
if __name__=="__main__":
  for name,P in [("H67.7 win",design(67.7)),("H60 win",design(60)),("H56 win",design(56)),("H52 win",design(52)),("H56 nowin",design(56,win=False)),
                 ("H60 win old U",design(60,U1=-22.8,U2=15.2)),("H60 win ut-17",design(60,ut=-17)),("H60 win ub 4",design(60,ub=4))]:
    r=k(P); print(f"{name:30s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]}",flush=True)
