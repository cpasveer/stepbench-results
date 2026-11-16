import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
from var4 import design
def d2(H,H1=None,**kw):
    P=design(H,**kw)
    if H1: 
        w=P['webs']; P['webs']=[(w[0][0],0,H1,1.2,[]),w[1]]
    return P
for name,P in [("H56 ut-17",design(56,ut=-17)),("H56 ut-18 ub10",design(56,ut=-18,ub=10)),("H52 ut-18 ub10",design(52,ut=-18,ub=10)),
               ("H54 ut-18 ub10",design(54,ut=-18,ub=10)),("H52 ut-18 ub10 W1 full",d2(52,67.7,ut=-18,ub=10)),("H56 ut-18 ub10 wtop 8",design(56,ut=-18,ub=10,wtop=8)),
               ("H56 ut-18 ub12",design(56,ut=-18,ub=12))]:
    r=k(P); print(f"{name:30s} k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} mode {r[3]}",flush=True)
