exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
LEG_END=38.5
def ov_leg(ru,rv,nm,s):
    r = r1 if nm=="S1" else r3
    root=CB+UB*ru+ZB*rv+NB*4.0; t=P0+AX*s+r*LEG_END
    L=bar(root,t,11.0)
    return round(sum(L.common(k).Volume for k in keep),3)
tests=[(2,-2,"S1",6),(0,-11,"S1",-6),(-1,-11,"S1",-6),(0,-10,"S1",-6),(4,-11.5,"S3",-2),(4,-11,"S3",-2),(3,-11,"S3",-2),(2,-11,"S3",-2),(4,-10,"S3",-2),(3,-10,"S3",-2)]
for t in tests: print("cand", t, ov_leg(*t))
