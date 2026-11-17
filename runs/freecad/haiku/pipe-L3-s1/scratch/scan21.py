exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
LEG_END=38.5
def ov_leg(ru,rv,nm,s,sec):
    r = r1 if nm=="S1" else r3
    root=CB+UB*ru+ZB*rv+NB*4.0; t=P0+AX*s+r*LEG_END
    L=bar(root,t,sec)
    return round(sum(L.common(k).Volume for k in keep),3)
tests=[((3.5,-12.5),"S3",-2,8.0),((3,-12.5),"S3",-2,8.0),((3.5,-12.5),"S3",-3,8.0),((3,-12),"S3",-2,8.0),((3,-12.5),"S3",-3,8.0)]
for (ru,rv),nm,s,sec in tests: print("cand", (ru,rv),nm,s,sec, ov_leg(ru,rv,nm,s,sec))
