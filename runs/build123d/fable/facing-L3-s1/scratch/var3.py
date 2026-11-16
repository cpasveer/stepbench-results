import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k, base, slotA
from var2 import webs
def W(H): return [(-22.2,0,H,1.2,[]),(14.6,0,H,1.2,[])]
V={
 "b1 B slot [-8.8,8.4]": base(openA=slotA,openB=[rect(-8.8,8.4,6,70)]),
 "b2 B window rim": base(openA=slotA,openB=[window(-8.8,8.4,6,62.1)]),
 "b3 B slot + X diag": base(openA=slotA,openB=[rect(-8.8,1.8,6,70)],diags=[(8,-15,0,67.7,1.2),(-15,8,0,67.7,1.2)]),
 "c1 H=45, B cut above 47": base(openA=slotA,openB=[rect(-23,8.4,47,70)],webs=W(45),diags=[(8,-15,0,45,1.2)]),
 "c2 H=52, B cut above 54": base(openA=slotA,openB=[rect(-23,8.4,54,70)],webs=W(52),diags=[(8,-15,0,52,1.2)]),
 "c3 H=58, B cut above 60": base(openA=slotA,openB=[rect(-23,8.4,60,70)],webs=W(58),diags=[(8,-15,0,58,1.2)]),
}
if __name__=="__main__":
  for n,P in V.items():
    r=k(P); print(f"{n:40s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]}",flush=True)
