exec(open('scratch/geo.py').read().split('import itertools')[0])
def armframe(Q1,Q2):
    d=(Q2-Q1)/np.linalg.norm(Q2-Q1); ew=np.cross(d,n); ew/=np.linalg.norm(ew); ed=np.cross(ew,d); return d,ew,ed
def clear(u0,v0,x1,th1,W,D):
    Q2=fromB(u0,v0,t);Q1=fromA(x1,th1,38);d,ew,ed=armframe(Q1,Q2); L=np.linalg.norm(Q2-Q1)
    res={}
    for b in P['resolved']['bolts']:
        p=np.array(b['position']);a=np.array(b['axis'])
        if b['id'][0]=='B': segs=[(t,t+5,4.0),(t+4,t+24,1.8)]
        else: segs=[(gA,gA+6,4.75),(gA+5,gA+30,2.3)]
        m=1e9
        for (s0,s1,r) in segs:
            # sample disc points
            for s in np.linspace(s0,s1,30):
                c=p+a*s
                # perpendicular basis
                e1=np.cross(a,[1,0,0]);
                if np.linalg.norm(e1)<.1: e1=np.cross(a,[0,1,0])
                e1/=np.linalg.norm(e1); e2=np.cross(a,e1)
                for ang in np.linspace(0,2*np.pi,24,endpoint=False):
                    q=c+r*(np.cos(ang)*e1+np.sin(ang)*e2)-Q1
                    al=q@d; 
                    if al<0 or al>L+20: continue
                    if (q+Q1-OB)@n < t: continue  # below pad top is pad
                    gw=abs(q@ew)-W/2; gd=abs(q@ed)-D/2
                    g=max(gw,gd) if (gw>0 or gd>0) else min(gw,gd)
                    if gw>0 and gd>0: g=np.hypot(gw,gd)
                    m=min(m,g)
        res[b['id']]=round(m,1)
    return res,np.degrees(np.arccos(-d@n))
if __name__=='__main__':
    import itertools
    out=[]
    for u0,v0,x1,th1 in itertools.product([-8,-6,-4,-2],[-2,-1,0,1,2,3],[-4,-2,0,2],[-6,-3,0]):
        r,a=clear(u0,v0,x1,th1,12,14); out.append((min(r.values()),a,u0,v0,x1,th1,r))
    out.sort(key=lambda o:-o[0])
    for o in out[:8]: print(o)
