exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
names=[]
for bid in BOLT: names+=[bid+"_head",bid+"_tool"]
for sec in [8.0,9.0]:
  for ru,rv in [(-6,0),(-5,0),(-4,0),(-6,-1),(-5,1),(-3,0)]:
    for s in [4,6]:
        root=CB+UB*ru+ZB*rv+NB*4.0; L=bar(root,P0+AX*s+r3*38.5,sec)
        tot=round(sum(L.common(kk).Volume for kk in keep),2)
        print("cand sec",sec,(ru,rv),"s",s,"overlap",tot)
