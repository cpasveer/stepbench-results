exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
names=[]
for bid in BOLT: names+=[bid+"_head",bid+"_tool"]
for ru,rv,s,sec in [(0,-17,6,11.0),(0,-17,4,10.0),(-4,-16,-6,11.0),(0,-17,-6,11.0)]:
    root=CB+UB*ru+ZB*rv+NB*4.0; L=bar(root,P0+AX*s+r3*38.5,sec)
    hits=[(names[i],round(L.common(kk).Volume,2)) for i,kk in enumerate(keep) if L.common(kk).Volume>0.01]
    print("cand",(ru,rv,s,sec),hits)
