import os, math
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
ns = {"__file__": os.path.join(H, "submission.py")}
exec(open(os.path.join(H, "submission.py")).read(), ns)
M_W = ns["M_WORLD_FROM_A"]; M_AB = ns["M_A_FROM_B"]
def fresh_A():  return Part.read(os.path.join(H,"output.step")).transformShape(M_W.inverse())
def fresh_obst():
    inv = M_W.inverse()
    return Part.read(os.path.join(H,"fixed_A.step")).transformShape(inv).fuse(Part.read(os.path.join(H,"fixed_B.step")).transformShape(inv))
def perp(d):
    d = V(d); d.normalize()
    t = V(1,0,0) if abs(d.x) < 0.9 else V(0,1,0)
    e1 = d.cross(t); e1.normalize(); e2 = d.cross(e1); e2.normalize()
    return d, e1, e2
bolts=[]
for x,y in [(30.0,0.0),(15.0,25.980762114),(-15.0,25.980762114),(-30.0,0.0),(-15.0,-25.980762114),(15.0,-25.980762114)]:
    bolts.append(("A", V(x,y,0), V(0,0,1), 13.0, 4.25, 33.0, 2.0, 70.0))
for x,y in [(-15,-40),(15,-40),(-15,40),(15,40)]:
    p = M_AB.multVec(V(x,y,0)); d = M_AB.multVec(V(0,0,1)) - M_AB.multVec(V(0,0,0))
    bolts.append(("B", p, d, 14.0, 5.0, 36.0, 2.5, 80.0))
for i,(name, p, d, htop, r_head, hleg, rleg, L) in enumerate(bolts):
    dn,e1,e2 = perp(d)
    base = p + dn*(htop - (5.0 if name=="A" else 6.0) + 0.01)
    room = Part.makeCylinder(r_head+0.5, (5.0 if name=="A" else 6.0)+0.99, base, dn)
    room_ov = round(fresh_A().common(room).Volume, 4)
    obst = fresh_A().fuse(fresh_obst())
    blocked = []
    for k in range(0,360,3):
        th = math.radians(k)
        rad = e1*math.cos(th) + e2*math.sin(th)
        leg = Part.makeCylinder(rleg, L, p + dn*hleg, rad)
        if leg.common(obst).Volume > 0.01: blocked.append(k)
    free = set(range(0,360,3)) - set(blocked)
    best=run=0
    seq = sorted(free)*2
    for j in range(len(seq)):
        run = run+1 if (j>0 and seq[j]==seq[j-1]+3) else 1
        best = max(best, run)
    print(i, name, "head_room_ov", room_ov, "blocked", len(blocked)*3, "free_arc", best*3, flush=True)
