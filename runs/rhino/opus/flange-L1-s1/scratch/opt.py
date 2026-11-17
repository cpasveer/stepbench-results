import json,numpy as np, itertools
T=json.load(open('task.json'));P=T['poses'][0]['resolved']
cA=np.array(P['mounting_faces'][0]['origin']);nA=np.array(P['mounting_faces'][0]['normal'])
cB=np.array(P['mounting_faces'][1]['origin']);nB=np.array(P['mounting_faces'][1]['normal'])
z=np.array([0,0,1.]);uA=np.cross(z,nA);uB=np.cross(z,nB)
tA=tB=8
def segdist(p1,p2,q1,q2,n=60):
    best=1e9
    for s in np.linspace(0,1,n):
        a=p1+(p2-p1)*s
        d=q2-q1; t=np.clip((a-q1)@d/(d@d),0,1)
        best=min(best,np.linalg.norm(a-(q1+d*t)))
    return best
def col(xa,za,xb,zb):
    A=cA+uA*xa+z*za+nA*(tA-1)
    # B end: point on B inner plane; param in B frame
    B=cB+uB*xb+z*zb+nB*(tB-1)
    return A,B
def score(xa,za,xb,zb):
    A,B=col(xa,za,xb,zb); m=1e9
    for b in P['bolts']:
        p=np.array(b['position']);a=np.array(b['axis']);t=tA if b['id'][0]=='A' else tB
        h=b['head_height'];R=b['head_diameter']/2+0.5
        m=min(m,segdist(A,B,p+a*t,p+a*(t+h+1))-R)
        m=min(m,segdist(A,B,p+a*(t+h),p+a*(t+h+22))-2.6)
    # inside plates
    m=min(m,36-np.hypot(xa,za), 21.5-abs(xb), 46.5-abs(zb))
    return m
import sys
if len(sys.argv)>1: print(score(*map(float,sys.argv[1:]))); sys.exit()
best=None
for xa in np.arange(-5,20,1.5):
  for za in np.arange(-15,10,1.5):
    for xb in np.arange(-21,0,1.5):
      for zb in np.arange(5,35,1.5):
        s=score(xa,za,xb,zb)
        if best is None or s>best[0]: best=(s,xa,za,xb,zb)
print(best)
