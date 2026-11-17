import json, math, sys
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
T = json.load(open(TD + "/task.json"))
POSE = T["poses"][0]; BOLTS = POSE["resolved"]["bolts"]
body = Part.read(TD + "/output.step").Solids[0]
fa = Part.read(TD + "/fixed_A.step"); fb = Part.read(TD + "/fixed_B.step")
GRIP = {"M5": None}
KEY = {"M5": dict(r=2.0, short=25.0, longl=70.0, sd=2.5), "M4": dict(r=1.5, short=20.0, longl=63.0, sd=2.0)}
HEAD = {"M5": (8.5/2, 5.0), "M4": (7.0/2, 4.0)}
def rot(v, axis, ang):
    return v*math.cos(ang) + axis.cross(v)*math.sin(ang) + axis*(axis.dot(v)*(1-math.cos(ang)))
def ortho(a):
    e = V(1,0,0) if abs(a.x)<0.9 else V(0,1,0)
    e = e - a*e.dot(a); e.normalize(); return e
def seg(p0, p1, r):
    d = p1 - p0
    dd = V(d); dd.normalize(); return Part.makeCylinder(r, d.Length, p0, dd)
def ball(p, r):
    return Part.makeSphere(r, p)
# head cylinders of all bolts (for obstacle list) - grip from placement: head base = surface pt + grip*axis
grips = {}
for b in BOLTS:
    grips[b["id"]] = None
def head_shape(b, grip):
    c = V(*b["position"]); a = V(*b["axis"])
    hd, hh = HEAD[b["size"]]
    base = c + a*grip
    return Part.makeCylinder(hd, hh, base, a)
# grip: distance from surface to head base is what we chose: A 8, B 6
GRIP_OF = {"A": 8.0, "B": 6.0}
heads = {}
for b in BOLTS:
    g = GRIP_OF[b["id"][0]]
    heads[b["id"]] = head_shape(b, g)
obst_static = body.fuse(fa).fuse(fb)
which = sys.argv[1:] or [b["id"] for b in BOLTS]
for b in BOLTS:
    if b["id"] not in which: continue
    g = GRIP_OF[b["id"][0]]
    k = KEY[b["size"]]
    c = V(*b["position"]); a = V(*b["axis"])
    head_top = c + a*(g + HEAD[b["size"]][1])
    tip = head_top - a*k["sd"]           # short leg tip in the socket
    others = [heads[o] for o in heads if o != b["id"]]
    obst = [body, fa, fb] + others
    e1 = ortho(a)
    res = {}
    for lean_deg, tilt_list in [(0.0,[0.0]), (25.0,[0,90,180,270])]:
        for tilt in tilt_list:
            for alpha in range(0, 360, 10):
                # base pose (lean, tilt) then swing about the bolt axis by alpha
                lam = math.radians(lean_deg)
                tv = rot(e1, a, math.radians(tilt))
                s0 = a*math.cos(lam) + tv*math.sin(lam)
                e0 = ortho(s0) if lean_deg>0 else e1
                # long leg direction: perpendicular to s0, start at e0
                e0 = e0 - s0*e0.dot(s0); e0.normalize()
                s = rot(s0, a, math.radians(alpha)); e = rot(e0, a, math.radians(alpha))
                bend = tip + s*k["short"]
                keyshape = seg(tip, bend, k["r"]).fuse(seg(bend, bend + e*k["longl"], k["r"]))
                d = min(keyshape.distToShape(o)[0] for o in obst)
                res[(lean_deg, tilt, alpha)] = round(d,2)
    # free sectors: 60 deg window (6 steps) with all clearances > 0.5
    out=[]
    for lean_deg, tilt_list in [(0.0,[0.0]),(25.0,[0,90,180,270])]:
        for tilt in tilt_list:
            ok=[]
            for st in range(0,360,10):
                win=[res[(lean_deg,tilt,(st+j)%360)] for j in range(0,70,10)]
                ok.append((st, min(win)))
            best=max(ok,key=lambda x:x[1])
            out.append((lean_deg,tilt,best[0],best[1], sum(1 for _,m in ok if m>0.5)))
    print(b["id"], "best sector per pose (lean,tilt,start,min clearance,#starts clear>0.5):")
    for o in out: print("   ", o)
    sys.stdout.flush()
