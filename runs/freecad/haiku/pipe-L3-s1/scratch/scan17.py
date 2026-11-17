exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
res=[]
for sec in [10.0,11.0]:
  for ru,rv in itertools.product([-8,-6,-4,-2,0],[-17,-16,-15]):
    for s in [4,5,6,7,8]:
        root=CB+UB*ru+ZB*rv+NB*4.0; L=bar(root,P0+AX*s+r3*38.5,sec)
        ov=round(sum(L.common(kk).Volume for kk in keep),2)
        res.append((ov,sec,(ru,rv),s))
res.sort(key=lambda x:x[0])
for r in res[:12]: print("cand",r)
