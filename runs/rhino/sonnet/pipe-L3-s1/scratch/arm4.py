from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0]);tg2=np.cross(a2,ex)
PLT=4.0
def section(w=11,ht=6,sb=2,psi=50):
    ap=sb+(w/2)*math.tan(math.radians(psi))
    return [(-w/2,-ht),(w/2,-ht),(w/2,sb),(0,ap),(-w/2,sb)]
def arm(Euv,xs,tt=0,w=11,abot=36.5,ntop=2.0,psi=50):
    Ps=np.array([xs,0,0])+38*a2+tt*tg2
    E=bp(Euv[0],Euv[1],PLT)
    D=(E-Ps);D/=np.linalg.norm(D)  # pad->plate
    g=-ex-(-ex@D)*D;g/=np.linalg.norm(g)
    e1=np.cross(D,g)
    sec=section(w,psi=psi)
    # place section so axis passes through Ps at section origin
    lo=[];up=[]
    for a,b in sec:
        p=Ps+a*e1+b*g
        tl=(abot-p@a2)/(D@a2); tu=(ntop-(p-c)@N)/(D@N)
        lo.append(p+tl*D);up.append(p+tu*D)
    return D,g,e1,lo,up
def clearance_pts(lo,up,n=25):
    out={}
    for t in np.linspace(0,1,n):
        pts=[(1-t)*l+t*u for l,u in zip(lo,up)]
        # sample polygon boundary + interior centroid
        cen=sum(pts)/len(pts)
        samp=pts+[cen]+[(pts[i]+pts[(i+1)%5])/2 for i in range(5)]
        for P in samp:
            for nm,p0,ax,r,l in cyl:
                q=P-p0;tt=q@ax
                if tt<0 or tt>l: continue
                d=np.linalg.norm(q-tt*ax)-r
                if nm not in out or d<out[nm]: out[nm]=d
    return out
if __name__=='__main__':
  for Euv in [(-13,1),(-13,3),(-14,0)]:
    for xs in [0,4,8]:
        D,g,e1,lo,up=arm(Euv,xs)
        upuv=[(round((p-c)@U,1),round((p-c)@V,1),round((p-c)@N,1)) for p in up]
        lox=[(round(p[0],1),round((p-38*a2)@tg2,1)) for p in lo]
        cl=clearance_pts(lo,up)
        print(Euv,xs,'D',D.round(2),'minclear',{k:round(float(v),1) for k,v in cl.items() if v<3})
        print('   up',upuv);print('   lo(x,tang)',lox)
