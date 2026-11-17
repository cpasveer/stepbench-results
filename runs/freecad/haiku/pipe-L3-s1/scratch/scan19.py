exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan13.py").read().split("best=[]")[0])
res=[]
for ru,rv in itertools.product([0,1,2,3,4,5],[-17.5,-17,-16.5,-16]):
    for s in [2,3,4,5]:
        for sec in [10.0]:
            root=CB+UB*ru+ZB*rv+NB*4.0; L=bar(root,P0+AX*s+r3*38.5,sec)
            ov=round(sum(L.common(kk).Volume for kk in keep),3)
            res.append((ov,sec,(ru,rv),s))
res.sort(key=lambda x:x[0])
for r in res[:10]: print("cand",r)
