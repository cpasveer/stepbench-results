exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan9.py").read().split('print("A:")')[0])
def leg_ok2(root_uv, tgt, sec=11.0):
    root=CB+UB*root_uv[0]+ZB*root_uv[1]+NB*4.0
    L=bar(root,tgt,sec)
    return round(sum(L.common(k).Volume for k in keep),2)
best=[]
for ru,rv in itertools.product([-4,-6,-8,-10],[-2,0,2,4]):
    for s in [4,5,6,7]:
        best.append((leg_ok2((ru,rv), P0+AX*s+r3*38.5), "C", (ru,rv), s))
for ru,rv in itertools.product([0,2,4,6],[-14,-15,-16,-17]):
    for s in [-4,-5,-6,-7]:
        best.append((leg_ok2((ru,rv), P0+AX*s+r3*38.5), "B", (ru,rv), s))
best.sort(key=lambda x:x[0])
for b in best[:12]: print("cand", b)
print("cand A check", leg_ok2((1,8), P0+r1*38.5), leg_ok2((2,8), P0+r1*38.5))
