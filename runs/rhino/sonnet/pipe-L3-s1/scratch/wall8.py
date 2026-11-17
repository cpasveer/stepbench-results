import ana
from ana import *
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B': p0=p0+ax*(9.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
E=bp(-28,0,0);Q1=np.array([E[1],E[2]]);Q0=np.array([-4.,30.])
d=Q1-Q0;L=np.linalg.norm(d);d/=L;l=np.array([-d[1],d[0]])
best=(9,None)
for s in np.linspace(0,L,120):
  for a in np.linspace(-8,8,17):
    yz=Q0+s*d+a*l
    for x in np.linspace(-23.5,-6,10):
        P=np.array([x,yz[0],yz[1]])
        if (P-c)@N<0.4 or np.hypot(*yz)<34: continue
        nm,p0,ax,r,ln=[cc for cc in ana.cyl if cc[0]=='B1'][0]
        q=P-p0;t=q@ax
        if t<0 or t>ln: continue
        dist=np.linalg.norm(q-t*ax)-r
        if dist<best[0]: best=(dist,(round((P-c)@U,1),round((P-c)@V,1),round((P-c)@N,1),round(t,1)))
print(best)
