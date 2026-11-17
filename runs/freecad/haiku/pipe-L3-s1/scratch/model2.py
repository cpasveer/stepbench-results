import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import *
E=2500.0; G=E/2.6
r1=A["A1"]; r3=A["A3"]
def S(r, s, rad=36): return P0 + s*AX + rad*r
plate = CB + 4*NB + (-4)*UB + (-4)*ZB   # plate attach point
def run(label, legs, a):
    pts=[plate]; fixed=set(); mem=[]
    for (sr, s) in legs:
        q = S(sr, s)
        pts.append(q); fixed.add(len(pts)-1)
        mem.append((0, len(pts)-1, a*a, a**4/12, 0.141*a**4))
    res=solve(pts, mem, fixed, 0, E, G)
    print(label, {k: round(1/v,1) for k,v in res.items()} , "min N/mm:", round(min(1/v for v in res.values()),1))
run("post-free tripod (S1 s=-6, S1 s=+6, S3 s=0)", [(r1,-6),(r1,6),(r3,0)], 10)
run("tripod (S1 s=0, S3 s=-8, S3 s=+8)", [(r1,0),(r3,-8),(r3,8)], 10)
run("tripod (S1 s=-6, S1 s=+6, S3 s=0), 12mm legs", [(r1,-6),(r1,6),(r3,0)], 12)
run("2 legs only (S1 s=0, S3 s=0), 10mm", [(r1,0),(r3,0)], 10)
