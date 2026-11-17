exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan.py").read().split("res=[]")[0])
names=[]
for bid in BOLT: names += [bid+"_head", bid+"_tool"]
for hu,hv,ht in [(-4,-4,28.0),(0,-12,28.0),(4,-4,28.0)]:
    b=build_var(hu,hv,ht).removeSplitter()
    print("cand",hu,hv,ht,[ (names[i], round(b.common(k).Volume,2)) for i,k in enumerate(keep) if b.common(k).Volume>0.01], "pipe", round(b.common(pipe).Volume,3))
