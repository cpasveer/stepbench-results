from ana import *
import math
def wallcheck(Q0,Q1,w,x0,x1,n_min=1.0):
    Q0=np.array(Q0);Q1=np.array(Q1);d=(Q1-Q0);L=np.linalg.norm(d);d/=L;l=np.array([-d[1],d[0]])
    worst={};cnt=0
    for s in np.linspace(0,L,60):
      for a in np.linspace(-w/2,w/2,9):
        yz=Q0+s*d+a*l
        for x in np.linspace(x0,x1,7):
            P=np.array([x,yz[0],yz[1]])
            if (P-c)@N<n_min: continue
            if np.hypot(*yz)<34.0: continue
            for nm,p0,ax,r,ln in cyl:
                q=P-p0;t=q@ax
                if t<0 or t>ln: continue
                dist=np.linalg.norm(q-t*ax)-r
                if nm not in worst or dist<worst[nm]: worst[nm]=dist
    return worst
if __name__=='__main__':
    E=c+(-13)*U
    Q1=(E[1],E[2])
    print('Q1',Q1)
    for Q0 in [(0,30),(4,30),(8,32)]:
      for w in (12,14):
        print(Q0,w,{k:round(float(v),1) for k,v in wallcheck(Q0,Q1,w,-23.5,-7.5).items() if v<4})
