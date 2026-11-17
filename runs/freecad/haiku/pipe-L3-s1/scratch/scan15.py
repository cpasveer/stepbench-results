exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
names=[]
for bid in BOLT: names+=[bid+"_head",bid+"_tool"]
for ru,rv,s in [(-2,14,6),(1,15,7),(-6,0,6),(-6,0,4)]:
    root=CB+UB*ru+ZB*rv+NB*4.0; L=bar(root,P0+AX*s+r3*38.5,11.0)
    hits=[(names[i],round(L.common(kk).Volume,2)) for i,kk in enumerate(keep) if L.common(kk).Volume>0.01]
    print("cand",(ru,rv,s),hits)
