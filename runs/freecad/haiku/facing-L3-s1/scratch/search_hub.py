import json, math
T = json.load(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/task.json"))["poses"][0]["resolved"]
A = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "A"]
B = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "B"]
def segdist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
    t = max(0, min(1, ((px-ax)*dx+(py-ay)*dy)/L2)) if L2 > 0 else 0
    qx, qy = ax+t*dx, ay+t*dy
    return math.hypot(px-qx, py-qy)
def score(P):
    # every arm (hub->own bolt) must keep >= M from the other set's bolt axes
    worst = 1e9; total = 0
    for own, other in ((A, B), (B, A)):
        for q in own:
            total += math.hypot(q[0]-P[0], q[1]-P[1])
            for o in other:
                worst = min(worst, segdist(o, P, q))
            # hub must clear all bolts too
    for q in A+B:
        worst = min(worst, math.hypot(q[0]-P[0], q[1]-P[1]) - 0.0)  # hub-to-bolt distance (informational)
    return worst, total
best = None
ca = (-126.0, 127.5); cb = (-124.18164228311065, 135.11206773573574)
for i in range(-60, 61):
    for j in range(-60, 61):
        P = (ca[0]+i*0.5, ca[1]+j*0.5)
        w, t = score(P)
        if w >= 6.0:
            # require hub distance to every bolt >= 14
            if min(math.hypot(q[0]-P[0], q[1]-P[1]) for q in A+B) < 14: continue
            if best is None or t < best[1]:
                best = (P, t, w)
print("best", best)
print("centres", ca, cb)
print("baseline P mid", score(((ca[0]+cb[0])/2, (ca[1]+cb[1])/2)))
for thr in (3.9, 4.5, 5.0, 5.5):
    best = None
    for i in range(-80, 81):
        for j in range(-80, 81):
            P = (ca[0]+i*0.5, ca[1]+j*0.5)
            w, t = score(P)
            if w >= thr and min(math.hypot(q[0]-P[0], q[1]-P[1]) for q in A+B) >= 14:
                if best is None or t < best[1]:
                    best = (P, t, w)
    print("thr", thr, "best", best)
