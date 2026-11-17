from wall import *
import ana
def wc2(Q0,Q1,w,x0,x1,Tn=7.0):
    Q0=np.array(Q0);Q1=np.array(Q1);d=(Q1-Q0);L=np.linalg.norm(d);d/=L;l=np.array([-d[1],d[0]])
    worst={}
    for s in np.linspace(0,L,70):
      for a in np.linspace(-w/2,w/2,9):
        yz=Q0+s*d+a*l
        for x in np.linspace(x0,x1,9):
            P=np.array([x,yz[0],yz[1]])
            nn=(P-c)@N
            if nn<1.0: continue
            if np.hypot(*yz)<34.0: continue
            # inside plate slab region? skip points that are within plate prism and below front plane
            for nm,p0,ax,r,ln in cyl:
                if nm[0]=='B': 
                    pass
                q=P-p0;t=q@ax
                if t<0 or t>ln: continue
                dist=np.linalg.norm(q-t*ax)-r
                if nm not in worst or dist<worst[nm]: worst[nm]=dist
    return worst
# B keepouts begin at plate front: t>=g (Bt=3.5 in cyl def); adjust cyl for B to start at 7.0
import ana
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B':
        p0=p0+ax*(7.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
import wall as W
W.cyl=ana.cyl
for ue in (-25,-17,-12):
  E=bp(ue,0,0);Q1=(E[1],E[2])
  for w in (12,14):
    for x1 in (-8,-12):
      wcx=W.wallcheck((-4,30),Q1,w,-23.5,x1,n_min=1.0)
      print(ue,w,x1,{k:round(float(v),1) for k,v in wcx.items() if v<3.5})
