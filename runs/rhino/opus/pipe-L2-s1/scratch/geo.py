import numpy as np, json, math
T=json.load(open('task.json'));P=T['poses'][0]
R=np.array(P['placements']['A']['rotation']);O=np.array(P['placements']['A']['origin'])
F=P['resolved']['mounting_faces'][0];OB=np.array(F['origin']);N=np.array(F['normal'])
W=np.array([0,0,1.]);U=np.cross(W,N);U/=np.linalg.norm(U)
def B2L(u,w,n): return R.T@(OB+u*U+w*W+n*N-O)
def L2B(p):
    q=O+R@p-OB; return np.array([q@U,q@W,q@N])
rhat=np.array([0,.374,.927]);rhat/=np.linalg.norm(rhat)
beta=math.radians(40)
uL=math.cos(beta)*np.array([1,0,0])+math.sin(beta)*rhat
print('u local',uL,'u in B coords',[uL@(R.T@U),uL@(R.T@W),uL@(R.T@N)])
R_OUT=41;PSI=math.radians(50);PHI=math.radians(35)
def inside_saddle(p,margin=0.5):
    y,z=p[1],p[2];r=math.hypot(y,z);phi=math.atan2(y,z)
    if r<34+margin or abs(phi)>PSI-math.radians(1): return False
    for s in (1,-1):
        nrm=np.array([s*math.sin(PHI),math.cos(PHI)])
        if nrm@[y,z]>R_OUT-margin: return False
    return True
import sys
gf=math.radians(float(sys.argv[1]));gb=math.radians(float(sys.argv[2]));L=float(sys.argv[3]);Lb=float(sys.argv[4])
Q3=np.array([-17,6.]);Q6=np.array([-17,0.])
Q4=Q3+L*np.array([-math.cos(gf),math.sin(gf)]);Q5=Q6+Lb*np.array([-math.cos(gb),math.sin(gb)])
for uu in np.linspace(float(sys.argv[5]),float(sys.argv[6]),5):
  for t in np.linspace(0,1,5):
    wn=Q4*(1-t)+Q5*t; p=B2L(uu,wn[0],wn[1])
    print(round(uu,1),round(t,2),np.round(p,1),inside_saddle(p), 'h',round(p@uL,1))
print('--- r/phi of bottom edge')
for uu in np.linspace(float(sys.argv[5]),float(sys.argv[6]),5):
  for t in (0,1):
    wn=Q4*(1-t)+Q5*t; p=B2L(uu,wn[0],wn[1])
    print(round(uu,1),t,'x',round(p[0],1),'r',round(math.hypot(p[1],p[2]),1),'phi',round(math.degrees(math.atan2(p[1],p[2])),1),'h',round(p@uL,1))
print('--- head zones vs keel')
def inkeel(b):
    u,w,n=b
    if w>-17: return False
    # back: n >= (-17-w)*tan(gb) ; front: n <= 6+(-17-w)*tan(gf)
    if n < (-17-w)*math.tan(gb): return False
    if n > 6+(-17-w)*math.tan(gf): return False
    drop=-17-w; Ltot=L*math.cos(gf)
    ul=-17+ (3+17)*min(drop/Ltot,1)
    return ul<=u<=17
for b in P['resolved']['bolts']:
    if not b['id'].startswith('A'): continue
    p=R.T@(np.array(b['position'])-O); a=R.T@np.array(b['axis'])
    e1=np.cross(a,[1,0,0]);e1/=np.linalg.norm(e1);e2=np.cross(a,e1)
    hits=0
    for s in np.linspace(7,40,12):
        for k in range(12):
            q=p+a*s+5.5*(math.cos(k*math.pi/6)*e1+math.sin(k*math.pi/6)*e2)
            if inkeel(L2B(q)): hits+=1
    print(b['id'],hits)
