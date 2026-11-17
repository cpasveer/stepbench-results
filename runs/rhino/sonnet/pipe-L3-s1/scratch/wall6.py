import wall4
from wall import *
import wall as W
E=bp(-14,0,0);Q0=np.array([-4.,30.]);Q1=np.array([E[1],E[2]])
d=Q1-Q0;L=np.linalg.norm(d);d/=L
print('L',L)
# find s where u=-30 along line (u of point at x arbitrary: u uses x too; approximate with x=-12)
for s in np.linspace(0,L,15):
    yz=Q0+s*d;P=np.array([-12,yz[0],yz[1]])
    print(round(s,1),'u=%.1f v=%.1f n=%.1f'%((P-c)@U,(P-c)@V,(P-c)@N))
def chk(s_step,w,xt1,xt2):
    out={}
    for (sa,sb,xt) in ((0,s_step,xt1),(s_step,L,xt2)):
        wc=W.wallcheck(tuple(Q0+sa*d),tuple(Q0+sb*d),w,-23.5,xt)
        for k,v in wc.items(): out[k]=min(out.get(k,9),round(float(v),1))
    return out
for ss in (L-30,L-26,L-22):
    print(ss,chk(ss,12,-6,-12.5))
