from ana import *
a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0]);tg=np.cross(a2,ex)
def go(Tuv,xs,w,h,abot=36.5,ntop=1.5):
    base=np.array([xs,0,0])+abot*a2
    Tm=bp(Tuv[0],Tuv[1],2.25)
    D=Tm-base;D/=np.linalg.norm(D)
    out=[]
    for sx in(-1,1):
      for st in(-1,1):
        p=base+sx*w/2*ex+st*h/2*tg
        r=[]
        for nn in(ntop,3.5):
            t=(nn-(p-c)@N)/(D@N); q=p+t*D; r.append((round((q-c)@U,1),round((q-c)@V,1)))
        out.append(r)
    print(Tuv,xs,'D.a',round(D@a2,2),'D.N',round(D@N,2),out)
    # clearance: sample interior hull points
    Pts=[]
    for s in np.linspace(0,1,30):
      pass
    return base,D
for T in [(-25,-15),(-25,-12),(-27,-15)]:
  go(T,1,11,11)
