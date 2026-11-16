import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
E=3200
def k(P):
    solve(build(P),'rbe3'); C=solve.C; m=metrics(C)
    S=np.diag([1,1,1,SPAN,SPAN,SPAN]); w,v=np.linalg.eigh(S@C@S)
    return E*m['6d'],E*m['t'],E*m['r'],np.round(v[:,-1],2)
def base(**kw):
    P=dict(TW=1.2,U1=-22.8,U2=15.2,webs=[(-22.2,0,67.7,1.2,[]),(14.6,0,67.7,1.2,[])],diags=[(8,-15,0,67.7,1.2)])
    P.update(kw); return P
slotA=[rect(-16,8-0.63-5.6,6,70)]
V={
 "full flanges, 1.2 webs full": base(),
 "A slotted": base(openA=slotA),
 "A slotted, H=45 all": base(openA=slotA,webs=[(-22.2,0,45,1.2,[]),(14.6,0,45,1.2,[])],diags=[(8,-15,0,45,1.2)]),
 "A slotted, H=30 all": base(openA=slotA,webs=[(-22.2,0,30,1.2,[]),(14.6,0,30,1.2,[])],diags=[(8,-15,0,30,1.2)]),
 "A slotted, TB 5": base(openA=slotA,TB=5.0),
 "A slotted, no diag": base(openA=slotA,diags=[]),
}
if __name__=="__main__":
  for n,P in V.items():
      r=k(P); print(f"{n:40s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]}",flush=True)
