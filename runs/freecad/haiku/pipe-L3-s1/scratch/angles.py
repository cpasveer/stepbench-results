import sys, json, numpy as np
sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import P0, AX, CB, NB, UB, ZB, A
r1=A["A1"]; r3=A["A3"]
def xyz(u,v,n): return CB+UB*u+ZB*v+NB*n
def uvn(p):
    d=p-CB; return np.round([d@UB, d@ZB, d@NB],2)
legs=[((2,-2),"S1",6),((0,-14),"S1",-6),((4,-16),"S3",-2)]
for (ru,rv),nm,s in legs:
    r = r1 if nm=="S1" else r3
    root=xyz(ru,rv,4.0); tgt=P0+AX*s+r*38.5
    d=(tgt-root); d=d/np.linalg.norm(d)
    ang_r=np.degrees(np.arccos(np.clip(d@r,-1,1)))
    ang_nb=np.degrees(np.arccos(np.clip(d@NB,-1,1)))
    print(nm, (ru,rv), "s",s, "leg angle to strap normal r: %.1f"%ang_r, " angle to NB: %.1f"%ang_nb, " landing uvn", uvn(tgt), "root uvn", uvn(root))
print("pipe clearance along each leg (distance from pipe axis; need >= 34):")
def rad_dist(p):
    d=p-P0; ax=d@AX; return float(np.linalg.norm(d-ax*AX))
for (ru,rv),nm,s in legs:
    r = r1 if nm=="S1" else r3
    root=xyz(ru,rv,4.0); tgt=P0+AX*s+r*38.5
    vals=[round(rad_dist(root+(tgt-root)*t),1) for t in np.linspace(0,1,9)]
    print(nm,(ru,rv),vals)
