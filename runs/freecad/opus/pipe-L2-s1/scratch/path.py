import numpy as np, json, itertools
exec(open('scratch/geo.py').read().split("print('B center")[0])
bu=RB[:,0];bv=RB[:,1]
rad=lambda p: np.sin(np.radians(p))*ey+np.cos(np.radians(p))*ez
bolts=T['poses'][0]['resolved']['bolts']
segs=[]
for b in bolts:
    p=np.array(b['position']);a=np.array(b['axis'])
    if b['id'][0]=='B': segs.append((b['id'],p+a*6,p+a*34))
    else: segs.append((b['id'],p+a*7,p+a*40))
def segdist(P,a,b):
    ab=b-a; t=np.clip(((P-a)@ab)/(ab@ab),0,1); return np.linalg.norm(P-(a+t[:,None]*ab),axis=1)
res=[]
ts=np.linspace(0,1,200)
for L1,L2,xs,ph in itertools.product([15,22,30,38],[10,20,30],[-4,0,4],[-10,-5,0,5,10]):
    p0=OB+2*n;p1=OB+(6+L1)*n;p3=OA+xs*ex+37*rad(ph);p2=OA+xs*ex+(43+L2)*rad(ph)
    C=((1-ts)**3)[:,None]*p0+(3*(1-ts)**2*ts)[:,None]*p1+(3*(1-ts)*ts**2)[:,None]*p2+(ts**3)[:,None]*p3
    md=min((segdist(C,a,b).min(),i) for i,a,b in segs)
    res.append((md[0],md[1],L1,L2,xs,ph))
res.sort(reverse=True)
for r in res[:10]: print(r)
