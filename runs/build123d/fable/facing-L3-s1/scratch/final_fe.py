import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
def cad(U1=-23.6,U2=15.4,H1=40.3,H2=57.0,ub=-17.0,ut=9.0,RIM=5.6,TW=1.2,win=True,HD=None,TB=3.8):
    HD=H2 if HD is None else HD
    hw=0.5*TW/math.sin(math.atan2(61.7,abs(ut-ub)))
    a_in=ub+hw+RIM; b_in=ut-hw-RIM; w_in=U2-TW-RIM
    oB=[rect(U1-1,b_in,H1,80),rect(b_in-0.01,w_in,H2,80)]
    if win: oB.append(window(U1+TW+RIM,b_in,6,H1-RIM))
    oA=[rect(a_in,w_in,6,80),rect(U1+TW+RIM,a_in+0.01,H2,80)]
    return dict(TW=TW,U1=U1,U2=U2,TB=TB,webs=[(U1+TW/2,0,H1,TW,[]),(U2-TW/2,0,H2,TW,[])],diags=[(ub,ut,0,HD,TW)],openA=oA,openB=oB)
if __name__=="__main__":
    import itertools
    for name,P in [("final",cad()),("H2 54",cad(H2=54)),("H2 60",cad(H2=60)),("nowin",cad(win=False)),("ut 10",cad(ut=10)),("ub -15",cad(ub=-15)),("H1 45",cad(H1=45))]:
        r=k(P); print(f"{name:20s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]}",flush=True)
