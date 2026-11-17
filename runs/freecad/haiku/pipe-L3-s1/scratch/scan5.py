exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan.py").read().split("res=[]")[0])
import itertools
out=[]
for hu,hv,ht in itertools.product([-4,-2,0,2,4],[10,12,14,16],[26.0,30.0,34.0]):
    b=build_var(hu,hv,ht).removeSplitter()
    ov=b.common(keepc).Volume
    pv=b.common(pipe).Volume
    out.append((round(ov,2), round(pv,2), hu, hv, ht))
clear=[o for o in out if o[0]<0.05]
print("clear:", len(clear), "of", len(out))
for o in sorted(out, key=lambda x:x[0])[:12]: print("cand", o)
