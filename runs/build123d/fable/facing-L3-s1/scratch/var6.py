import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
def des(H1,H2,HD,U1=-24.0,U2=15.6,ub=10,ut=-18,win=True,wtop=5.6,rev=False,Hf=None):
    # B flange: u in [U1, cut] up to H1 ; rest up to max(H2,HD)
    oB=[]
    if not rev:
        cut = ut-0.63-0.01 if False else U1+1.2+5.6    # flange beside W1 kept RIM wide above H1? no: cut everything left of diag strip
        if H1<67.7: oB.append(rect(U1-1, ut-0.66-5.6 if (ut-0.66-5.6)>U1+1.2 else U1+1.2, H1, 70) if False else rect(U1-1,U1+1.2+ (0),H1,70))
        if H1<67.7: oB.append(rect(U1-1,ut-0.66,H1,70))
        if win: oB.append(window(ut+0.66+5.6,U2-6.8,6,min(H2,HD)-wtop))
        oA=[rect(U1+6.8,ub-0.66-5.6,6,70)]
    else:
        # diag head near W2 (ut>0), foot near W1 (ub<0)
        if H1<67.7: oB.append(rect(U1-1,ut-0.66-5.6,H1,70))   # keep nothing left above H1 except right part
        if win: oB.append(window(U1+6.8,ut-0.66-5.6,6,H1-wtop))
        oA=[rect(ub+0.66+5.6,U2-6.8,6,70)]
    return dict(TW=1.2,U1=U1,U2=U2,webs=[(U1+0.6,0,H1,1.2,[]),(U2-0.6,0,H2,1.2,[])],diags=[(ub,ut,0,HD,1.2)],openA=oA,openB=oB)
if __name__=="__main__":
  for name,P in [("W1 45, W2/D full",des(45,67.7,67.7)),("W1 56, W2/D full",des(56,67.7,67.7)),("W1 45, W2/D 62",des(45,62,62)),
               ("rev W1 56 D/W2 full",des(56,67.7,67.7,ub=-17,ut=9,rev=True)),("rev W1 45 D/W2 full",des(45,67.7,67.7,ub=-17,ut=9,rev=True)),
               ("all full",des(67.7,67.7,67.7))]:
    G=build(P); r=k(P); print(f"{name:30s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]} cells {int((G>0).sum())}",flush=True)
