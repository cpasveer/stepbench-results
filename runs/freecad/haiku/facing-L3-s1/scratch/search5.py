import json, math
exec(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/scratch/search2.py").read().split("ca = ")[0])
ca = (-126.0, 127.5)
best_by = {}
for thr in (5.5, 6.0, 6.5, 7.0):
    res = []
    for i in range(-130, 131, 2):
        for j in range(-130, 131, 2):
            P = (ca[0]+i*0.5, ca[1]+j*0.5)
            reachB = [math.hypot(q[0]-P[0], q[1]-P[1]) for q in B]
            if max(reachB) > 64: continue
            if min(math.hypot(q[0]-P[0], q[1]-P[1]) for q in A+B) < 14: continue
            w = min([segdist(o, P, q) for q in A for o in B] + [segdist(o, P, q) for q in B for o in A])
            if w < thr: continue
            total = sum(math.hypot(q[0]-P[0], q[1]-P[1]) for q in A) + sum(reachB)
            res.append((round(total,1), P, round(w,2), round(max(reachB),1)))
    res.sort()
    print(thr, res[0] if res else None)
