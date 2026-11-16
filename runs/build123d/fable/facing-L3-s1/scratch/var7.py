import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
from var6 import des
def vol(H1,H2,HD,ub,ut,U1=-24.0,U2=15.6):
    L=math.hypot(61.7,ut-ub)
    return (74*H1+74*H2+1.2*L*HD)/1000
for (H1,H2) in [(41,56),(41,60),(41,64),(45,56),(45,60),(50,56),(50,60),(41,67.7)]:
    for (ub,ut) in [(-17,9)]:
        P=des(H1,H2,H2,ub=ub,ut=ut,rev=True); r=k(P)
        print(f"rev H1 {H1} H2 {H2} ub {ub} ut {ut}: k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]} walls {vol(H1,H2,H2,ub,ut):.2f}",flush=True)
