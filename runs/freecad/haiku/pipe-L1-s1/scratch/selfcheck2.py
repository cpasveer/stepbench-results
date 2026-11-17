import json, os, math
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
B = {b["id"]: b for b in T["poses"][0]["resolved"]["bolts"]}
s = Part.read(os.path.join(HERE, "output.step"))
GRIP = 8.0
for bid, b in B.items():
    p = V(*b["position"]); ax = V(*b["axis"]); hd = b["head_diameter"]; hh = b["head_height"]
    base = p + ax * GRIP
    cyl = Part.makeCylinder(hd/2 + 0.5, hh + 1.0, base, ax)
    print(bid, "head-clearance overlap", round(s.common(cyl).Volume, 5))
    # approximate key swing: free angular sectors at height above head top
    e1 = ax.cross(V(1,0,0)) if abs(ax.x) < 0.9 else ax.cross(V(0,1,0))
    e1.normalize(); e2 = ax.cross(e1); e2.normalize()
    h = GRIP + hh + 1.5
    free = []
    for a in range(0, 360, 2):
        c, sn = math.cos(math.radians(a)), math.sin(math.radians(a))
        hit = False
        for k in range(0, 36):
            r = hd/2 + 0.5 + k * (70 - hd/2 - 0.5) / 35.0
            q = p + ax * h + (e1 * c + e2 * sn) * r
            if s.isInside(q, 1e-6, True):
                hit = True; break
        free.append(not hit)
    # longest run of free angles (circular)
    best = cur = 0
    for f in free + free:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    print(bid, "longest free sector (deg)", min(best*2, 360), "free fraction", round(sum(free)/len(free), 2))
