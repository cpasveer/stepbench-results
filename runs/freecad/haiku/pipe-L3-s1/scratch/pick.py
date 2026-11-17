import sys, pickle, random, itertools
import numpy as np
sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import beam_k, P0, AX, CB, NB, UB, ZB, A
E=2500.0; G=E/2.6
T=__import__("json").load(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/task.json"))
r1=A["A1"]; r3=A["A3"]
def xyz(u,v,n): return CB+UB*u+ZB*v+NB*n
cands=pickle.load(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/cands.pkl","rb"))
cands=[(float(a),float(b),nm,float(s)) for a,b,nm,s in cands]
def target(nm,s):
    r = r1 if nm=="S1" else r3
    return P0+AX*s+r*38.5
def minstiff(pts, mem, fixed, load=0):
    n=len(pts); K=np.zeros((6*n,6*n))
    for (i,j,Aa,I,J) in mem:
        ke=beam_k(pts[i],pts[j],E,G,Aa,I,I,J)
        dof=list(range(6*i,6*i+6))+list(range(6*j,6*j+6)); K[np.ix_(dof,dof)]+=ke
    free=[d for d in range(6*n) if d//6 not in fixed]
    Kff=K[np.ix_(free,free)]; Cm=np.zeros((3,3))
    for a_ in range(3):
        for b_ in range(3):
            F=np.zeros(len(free)); F[free.index(6*load+a_)]=1
            Cm[a_,b_]=np.linalg.solve(Kff,F)[free.index(6*load+b_)]
    try:
        w=np.linalg.eigvalsh(np.linalg.inv(Cm))
        return float(w.min())
    except Exception:
        return 0.0
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
def ok_spacing(legs):
    for a,b in itertools.combinations(legs,2):
        dr=np.hypot(a[0]-b[0],a[1]-b[1])
        ta=target(a[2],a[3]); tb=target(b[2],b[3])
        dt=np.linalg.norm(ta-tb)
        if dr<8 and dt<9: return False
        if dr<4: return False
    return True
random.seed(1)
best=[]
# Only triples with at least one S1 and one S3 leg (non-coplanar tripod)
S1c=[c for c in cands if c[2]=="S1"]; S3c=[c for c in cands if c[2]=="S3"]
print("S1 legs", len(S1c), "S3 legs", len(S3c), flush=True)
seen=set()
for it in range(6000):
    legs=[random.choice(S1c), random.choice(S1c)] if random.random()<0.5 else [random.choice(S1c), random.choice(S3c)]
    if legs[0]==legs[1]: continue
    legs.append(random.choice(S3c) if random.random()<0.6 else random.choice(S1c))
    key=tuple(sorted(legs))
    if key in seen or len(set(key))<3: continue
    seen.add(key)
    if not ok_spacing(legs): continue
    best.append((evaluate(legs), legs))
best.sort(key=lambda x:-x[0])
for val,legs in best[:8]:
    print("stiff %.1f" % val, [(l[0],l[1],l[2],l[3]) for l in legs], flush=True)
