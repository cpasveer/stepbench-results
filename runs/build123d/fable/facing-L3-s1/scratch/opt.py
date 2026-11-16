import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
def cad(U1=-23.6,U2=15.4,H1=40.3,H2=57.0,ub=-17.0,ut=9.0,RIM=5.6,TW=1.2,win=True,H1w=None,HD=None,H2w=None):
    H1w=H1 if H1w is None else H1w; HD=H2 if HD is None else HD; H2w=H2 if H2w is None else H2w
    L=math.hypot(61.7,ut-ub); hw=0.5*TW*L/61.7
    a_in=ub+hw+RIM; b_in=ut-hw-RIM; w_in=U2-TW-RIM
    oB=[rect(U1-1,b_in,H1,80),rect(b_in-0.01,w_in,H2,80)]
    wa=0
    if win:
        oB.append(window(U1+TW+RIM,b_in,6,H1-RIM)); ww=b_in-(U1+TW+RIM); rise=0.5*ww*SL; wa=ww*(H1-RIM-rise-6)+0.5*ww*rise
    oA=[rect(a_in,w_in,0,80),rect(U1+TW+RIM,a_in+0.01,H2,80)]
    P=dict(TW=TW,U1=U1,U2=U2,webs=[(U1+TW/2,0,H1w,TW,[]),(U2-TW/2,0,H2w,TW,[])],diags=[(ub,ut,0,HD,TW)],openA=oA,openB=oB)
    vol=(61.7*TW*(H1w+H2w)+L*TW*HD + 3.8*((b_in-U1)*H1-wa+(U2-b_in)*H2+(U2-w_in)*(67.7-H2)))/1000
    return P,vol
if __name__=="__main__":
    cases={
     "ref": {},
     "ub-19 ut10.5": dict(ub=-19,ut=10.5),
     "ub-19 ut10.5 H2 54": dict(ub=-19,ut=10.5,H2=54),
     "ub-19 ut10.5 H2 51": dict(ub=-19,ut=10.5,H2=51),
     "ub-19 ut10.5 H2 51 H1w 46": dict(ub=-19,ut=10.5,H2=51,H1w=46),
     "ub-19 ut10.5 H2 48 H1w 48": dict(ub=-19,ut=10.5,H2=48,H1w=48),
     "ub-19 ut10.5 H2 51 H1w 34": dict(ub=-19,ut=10.5,H2=51,H1w=34),
     "ub-19 ut10.5 H2 54 HD 46": dict(ub=-19,ut=10.5,H2=54,HD=46),
     "ub-19 ut10.5 H2 51 nowin": dict(ub=-19,ut=10.5,H2=51,win=False),
    }
    for name,kw in cases.items():
        P,vol=cad(**kw); r=k(P); print(f"{name:32s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} vol(walls+Bfl) {vol:5.2f} mode {r[3]}",flush=True)
    P,vol=cad(ub=-19,ut=10.5,H2=56,HD=54,H1w=35); r=k(P); print("RUN5",r[0],r[1],r[2],vol,r[3])
