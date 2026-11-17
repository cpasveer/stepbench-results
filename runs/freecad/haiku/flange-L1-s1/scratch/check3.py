import os, math
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
ns = {"__file__": os.path.join(H, "submission.py")}
exec(open(os.path.join(H, "submission.py")).read(), ns)
M_W = ns["M_WORLD_FROM_A"]; M_AB = ns["M_A_FROM_B"]
MB_W = M_W.multiply(M_AB)  # B-local -> world
S = Part.read(os.path.join(H,"output.step"))
obst = Part.read(os.path.join(H,"fixed_A.step")).fuse(Part.read(os.path.join(H,"fixed_B.step")))
print("valid", S.isValid(), "solids", len(S.Solids), "vol", round(S.Volume,1))
print("overlap", round(S.common(obst).Volume,5))
def perp(d):
    d = V(d); d.normalize()
    t = V(1,0,0) if abs(d.x) < 0.9 else V(0,1,0)
    e1 = d.cross(t); e1.normalize(); e2 = d.cross(e1); e2.normalize()
    return d, e1, e2
bolts=[]
for x,y in [(30.0,0.0),(15.0,25.980762114),(-15.0,25.980762114),(-30.0,0.0),(-15.0,-25.980762114),(15.0,-25.980762114)]:
    p = M_W.multVec(V(x,y,0)); d = M_W.multVec(V(0,0,1)) - M_W.multVec(V(0,0,0))
    bolts.append(("A", p, d, 5.0, 4.25, 33.0-8.0, 2.0, 70.0))
for x,y in [(-15,-40),(15,-40),(-15,40),(15,40)]:
    p = MB_W.multVec(V(x,y,0)); d = MB_W.multVec(V(0,0,1)) - MB_W.multVec(V(0,0,0))
    bolts.append(("B", p, d, 6.0, 5.0, 36.0-8.0, 2.5, 80.0))
for i,(name, p, d, H_head, r_head, hleg_above_pad, rleg, L) in enumerate(bolts):
    dn,e1,e2 = perp(d)
    pad = 8.0
    base = p + dn*(pad + 0.01)
    room = Part.makeCylinder(r_head+0.5, H_head+1.0-0.01, base, dn)
    room_ov = round(S.common(room).Volume, 4)
    blocked=[]
    for k in range(0,360,3):
        th = math.radians(k)
        rad = e1*math.cos(th) + e2*math.sin(th)
        leg = Part.makeCylinder(rleg, L, p + dn*(pad + hleg_above_pad), rad)
        if leg.common(S).Volume > 0.01 or leg.common(obst).Volume > 0.01:
            blocked.append(k)
    free = sorted(set(range(0,360,3)) - set(blocked))
    best=run=0
    seq = free+free
    for j in range(len(seq)):
        run = run+1 if (j>0 and seq[j]==seq[j-1]+3) else 1
        best = max(best, run)
    best = min(best, len(free))
    print(i, name, "head_room_ov", room_ov, "blocked_deg", len(blocked)*3, "free_arc_deg", best*3 if free else 0, flush=True)
