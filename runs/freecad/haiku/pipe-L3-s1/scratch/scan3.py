exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan.py").read().split("res=[]")[0])
import itertools
out=[]
for hu,hv,ht in itertools.product([6,8,10,12,14],[-8,-6,-4,-2,0,2],[28.0,32.0]):
    b=build_var(hu,hv,ht).removeSplitter()
    ov=b.common(keepc).Volume
    pv=b.common(pipe).Volume
    out.append((round(ov,2), round(pv,2), hu, hv, ht))
clear=[o for o in out if o[0]<0.05]
print("clear(keep-out<0.05):", len(clear))
for o in sorted(clear, key=lambda x:x[1])[:30]: print("cand", o)
