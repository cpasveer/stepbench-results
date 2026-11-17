import sys, pickle, random, itertools
import numpy as np
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/pick.py").read().split("cands=pickle.load")[0])
cands=pickle.load(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/cands2.pkl","rb"))
cands=[(float(a),float(b),nm,float(s)) for a,b,nm,s in cands]
S1c=[c for c in cands if c[2]=="S1"]; S3c=[c for c in cands if c[2]=="S3"]
print("S1", len(S1c), "S3", len(S3c), flush=True)
def target(nm,s):
    r = r1 if nm=="S1" else r3
    return P0+AX*s+r*38.5
def evaluate(legs, sec=11.0):
    pts=[xyz(0,0,4)]; mem=[]; fixed=set(); rn={}
    for (ru,rv,nm,s) in legs:
        if (ru,rv) not in rn:
            pts.append(xyz(ru,rv,4)); rn[(ru,rv)]=len(pts)-1
            mem.append((0,rn[(ru,rv)],1600.0,1600*400/12.0,2*1600*400/12.0))
        pts.append(target(nm,s)); fixed.add(len(pts)-1)
        mem.append((rn[(ru,rv)],len(pts)-1,sec*sec,sec**4/12,0.141*sec**4))
    try:
        return minstiff(pts,mem,fixed)
    except Exception:
        return 0.0
def ok(legs):
    for a,b in itertools.combinations(legs,2):
        dr=np.hypot(a[0]-b[0],a[1]-b[1]); dt=np.linalg.norm(target(a[2],a[3])-target(b[2],b[3]))
        if dr<6 or (dr<8 and dt<9): return False
    return True
random.seed(2)
best=[]; seen=set()
for it in range(20000):
    if random.random()<0.5: legs=[random.choice(S1c), random.choice(S1c), random.choice(S3c)]
    else: legs=[random.choice(S1c), random.choice(S3c), random.choice(S3c)]
    key=tuple(sorted(legs))
    if key in seen or len(set(legs))<3: continue
    seen.add(key)
    if not ok(legs): continue
    best.append((evaluate(legs), legs))
    if len(best)>3000: break
best.sort(key=lambda x:-x[0])
for val,legs in best[:6]:
    print("stiff %.1f"%val, [(l[0],l[1],l[2],l[3]) for l in legs], flush=True)
