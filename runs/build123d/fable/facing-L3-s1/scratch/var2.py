import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k, base, slotA
def house(a,b,z1,z2):   # in (h, zlocal)
    rise=0.5*(z2-z1)*SL
    return [(a,z1),(a,z2),(b,z2),(b+rise,0.5*(z1+z2)),(b,z1)]
def webs(wins1,wins2,H=67.7): return [(-22.2,0,H,1.2,wins1),(14.6,0,H,1.2,wins2)]
w1=house(10,30,8,53.7); w2=house(8,34,6,55.7); w3=house(10,40,14,47.7)
V={
 "win(10,30,8..53.7) both": base(openA=slotA,webs=webs([w1],[w1])),
 "win(8,34,6..55.7) both": base(openA=slotA,webs=webs([w2],[w2])),
 "win(10,40,14..47.7) both": base(openA=slotA,webs=webs([w3],[w3])),
 "win w1 both + diag H=45": base(openA=slotA,webs=webs([w1],[w1]),diags=[(8,-15,0,45,1.2)]),
 "no win, diag H=30": base(openA=slotA,diags=[(8,-15,0,30,1.2)]),
 "no win, diag h 20..50": base(openA=slotA,diags=[(8,-15,20,50,1.2)]),
}
if __name__=="__main__":
  for n,P in V.items():
    r=k(P); print(f"{n:40s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]}",flush=True)
