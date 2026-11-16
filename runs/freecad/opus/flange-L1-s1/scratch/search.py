import json,numpy as np, itertools
T=json.load(open('task.json'));P=T['poses'][0]
def fr(n):
    p=P['placements'][n];R=np.array(p['rotation']);return np.array(p['origin']),R
oA,RA=fr('A');oB,RB=fr('B')
TA=TB=8
bolts=[(np.array(b['position']),np.array(b['axis'])) for b in P['resolved']['bolts']]
def seg_dist(p,d,L,a,b):
    # min distance between bolt segment p+d*t t in[0,L] and segment a-b
    best=1e9
    for t in np.linspace(0,L,60):
        q=p+d*t; ab=b-a; s=np.clip(np.dot(q-a,ab)/np.dot(ab,ab),0,1)
        best=min(best,np.linalg.norm(q-(a+ab*s)))
    return best
res=[]
for ax,ay in itertools.product(np.arange(-12,13,3),repeat=2):
  a=oA+RA@np.array([ax,ay,TA])
  for bx,by in itertools.product(np.arange(-6,7,3),np.arange(-24,25,3)):
    b=oB+RB@np.array([bx,by,TB])
    ds=[seg_dist(p,d,42,a,b) for p,d in bolts]
    res.append((min(ds),ax,ay,bx,by,np.round(ds,1)))
res.sort(key=lambda r:-r[0])
for r in res[:8]: print(r)
