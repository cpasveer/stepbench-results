exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan.py").read().split("res=[]")[0])
names=[]
for bid in BOLT: names += [bid+"_head", bid+"_tool"]
for hu,hv,ht in [(10,-5,28.0),(12,-4,32.0),(10,-5,36.0),(14,-2,32.0)]:
    b=build_var(hu,hv,ht).removeSplitter()
    hits=[(names[i], round(b.common(k).Volume,2)) for i,k in enumerate(keep) if b.common(k).Volume>0.01]
    print("cand",hu,hv,ht,hits,"pipe",round(b.common(pipe).Volume,2))
