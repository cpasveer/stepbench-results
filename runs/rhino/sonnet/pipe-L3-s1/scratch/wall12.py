import ana
from ana import *
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B': p0=p0+ax*(9.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
def chk(ue,wbase,wtop,top,xref=-20.5):
    E=bp(ue,0,0);Q1=np.array([E[1],E[2]]);Q0=np.array([-4.,30.])
    d=Q1-Q0;L=np.linalg.norm(d);d/=L;l=np.array([-d[1],d[0]])
    worst={}
    for s in np.linspace(0,L,80):
      for x in np.linspace(xref,top,12):
        w=wtop+(wbase-wtop)*(top-x)/(top-xref)
        for a in np.linspace(-w/2,w/2,13):
            yz=Q0+s*d+a*l
            P=np.array([x,yz[0],yz[1]])
            if (P-c)@N<0.4 or np.hypot(*yz)<34: continue
            for nm,p0,ax,r,ln in ana.cyl:
                q=P-p0;t=q@ax
                if t<0 or t>ln: continue
                dist=np.linalg.norm(q-t*ax)-r
                if nm not in worst or dist<worst[nm]: worst[nm]=dist
    return {k:round(float(v),2) for k,v in worst.items() if v<3}
for top in (-3,-2,0):
    for wt in (12,14):
        print('end -28 top',top,'wbase 24 wtop',wt,chk(-28,24,wt,top))
print('----')
for ue in (-28,-30,-32):
    for top in (-4,-3):
        print('end',ue,'top',top,chk(ue,24,12,top))
