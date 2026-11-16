import sys; sys.path.insert(0,"scratch")
import numpy as np
from fe import *
from var import k
from opt import cad
for H2 in (54,55.5,57):
    for H1w in (28,34,40.3):
        P,vol=cad(ub=-19,ut=10.5,H2=H2,H1w=H1w); r=k(P)
        print(f"H2 {H2} H1w {H1w}: k6d {r[0]:6.1f} kt {r[1]:6.1f} kr {r[2]:6.1f} vol {vol:5.2f} mode {r[3]}",flush=True)
