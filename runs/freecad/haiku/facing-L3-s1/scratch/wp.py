import json, math
T = json.load(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/task.json"))["poses"][0]["resolved"]
bolts = {b["id"]: tuple(b["position"][:2]) for b in T["bolts"]}
Ca = (-126.0, 127.5); th = math.acos(0.5735765984126814); 
c0 = (0.5735765984126814, -0.8191521213976911); c1 = (0.8191521213976911, 0.5735765984126814)
def local(p):
    d = (p[0]-Ca[0], p[1]-Ca[1]); return (d[0]*c0[0]+d[1]*c0[1], d[0]*c1[0]+d[1]*c1[1])
def segdist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
    t = max(0, min(1, ((px-ax)*dx+(py-ay)*dy)/L2))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))
P = (-124.18164228311065, 135.11206773573574)
B3 = bolts["B3"]; A3 = bolts["A3"]
best = None
for i in range(-120, 121):
    for j in range(-120, 121):
        W = (-100 + i*0.25, 120 + j*0.25)
        lu, lv = local(W)
        if max(abs(lu), abs(lv)) > 32.0: continue
        d1 = segdist(B3, P, W); d2 = segdist(B3, W, A3)
        if min(d1, d2) < 6.8: continue
        score = math.hypot(W[0]-P[0], W[1]-P[1]) + math.hypot(A3[0]-W[0], A3[1]-W[1])
        if best is None or score < best[0]: best = (score, W, round(d1,2), round(d2,2), round(lu,1), round(lv,1))
print(best)
