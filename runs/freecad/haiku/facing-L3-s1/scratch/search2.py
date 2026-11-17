import json, math
T = json.load(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/task.json"))["poses"][0]["resolved"]
A = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "A"]
B = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "B"]
def segdist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
    t = max(0, min(1, ((px-ax)*dx+(py-ay)*dy)/L2))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))
ca = (-126.0, 127.5)
res = []
for i in range(-120, 121):
    for j in range(-120, 121):
        P = (ca[0]+i*0.5, ca[1]+j*0.5)
        reachB = [math.hypot(q[0]-P[0], q[1]-P[1]) for q in B]
        if max(reachB) > 38: continue
        if min(math.hypot(q[0]-P[0], q[1]-P[1]) for q in A+B) < 14: continue
        w = min([segdist(o, P, q) for q in B for o in A] + [segdist(o, P, q) for q in A for o in B])
        if w < 4.5: continue
        total = sum(math.hypot(q[0]-P[0], q[1]-P[1]) for q in A) + sum(reachB)
        res.append((total, P, w, max(reachB)))
res.sort()
for r in res[:5]: print(r)
print("count", len(res))
