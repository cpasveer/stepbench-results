import json, numpy as np
T=json.load(open('task.json'));P=T['poses'][0]
RA=np.array(P['placements']['A']['rotation']);OA=np.array(P['placements']['A']['origin'])
RB=np.array(P['placements']['B']['rotation']);OB=np.array(P['placements']['B']['origin'])
xA,yA,zA=RA[:,0],RA[:,1],RA[:,2]; u,v,n=RB[:,0],RB[:,1],RB[:,2]
def toA(p): d=p-OA; return np.array([d@xA,d@yA,d@zA])
def toB(p): d=p-OB; return np.array([d@u,d@v,d@n])
for b in P['resolved']['bolts']:
    p=np.array(b['position']); print(b['id'],toA(p).round(2),toB(p).round(2))
print('B in A',toA(OB).round(2), 'n in A',np.array([n@xA,n@yA,n@zA]).round(3))

def segdist(p1,p2,q1,q2,N=200):
    ts=np.linspace(0,1,N); A=p1+np.outer(ts,p2-p1); B=q1+np.outer(ts,q2-q1)
    return np.min(np.linalg.norm(A[:,None,:]-B[None,:,:],axis=2))
t=7.0; gA=5.0
def fromB(uu,vv,hh): return OB+uu*u+vv*v+hh*n
def fromA(x,th,r): th=np.radians(th); return OA+x*xA+r*(np.cos(th)*zA+np.sin(th)*yA)
import itertools
best=[]
for u0,v0,x1,th1 in itertools.product(np.arange(-8,9,2),np.arange(-6,7,2),np.arange(-14,4,2),np.arange(-6,16,3)):
    Q2=fromB(u0,v0,t); Q1=fromA(x1,th1,38)
    d=(Q2-Q1)/np.linalg.norm(Q2-Q1)
    angB=np.degrees(np.arccos(-d@n))
    rad=np.cos(np.radians(th1))*zA+np.sin(np.radians(th1))*yA
    angA=np.degrees(np.arccos(d@rad))
    md=[]
    for b in P['resolved']['bolts']:
        p=np.array(b['position']);a=np.array(b['axis'])
        if b['id'][0]=='B': s0=p+a*(t+4); s1=p+a*(t+4+20)
        else: s0=p+a*(gA+5); s1=p+a*(gA+5+25)
        md.append(segdist(Q1,Q2+d*0,s0,s1))
    best.append((min(md),angB,angA,u0,v0,x1,th1,[round(x,1) for x in md]))
best=[b for b in best if b[1]<40 and b[2]<35]
best.sort(key=lambda b:-b[0])
for b in best[:10]: print(np.round(b[:7],1),b[7])
