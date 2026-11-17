import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import *
E=2500.0; G=E/2.6
def lever_pt(): return CB
# nodes: 0 plate mount (centre at n=4), 1 post top, 2 strap1 end, 3 strap3 end (fixed)
r1=A["A1"]; r3=A["A3"]
S1 = P0 + 36*r1; S3 = P0 + 36*r3
plate = CB + 4*NB
top = CB - 4*UB - 4*ZB + 48*NB
nodes=[plate, top, S1, S3]
a=10.0; I=a**4/12; J=0.141*a**4
mem=[(0,1,a*a,I,J),(1,2,a*a,I,J),(1,3,a*a,I,J)]
res=solve(nodes, mem, fixed={2,3}, load_node=0, E=E, G=G)
print({k: round(v,4) for k,v in res.items()})
