from ana import *
a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0]);tg=np.cross(a2,ex)
def arm(Tuv,xs,w,h,ntop=1.5,abot=36.5):
    Ps=np.array([xs,0,0])+38*a2
    Tm=bp(Tuv[0],Tuv[1],2.25)
    d=Tm-Ps;d/=np.linalg.norm(d)
    e1=np.cross(d,ex);e1/=np.linalg.norm(e1)
    e2=np.cross(d,e1)
    verts={}
    for s1 in(-1,1):
      for s2 in(-1,1):
        o=Ps+s1*w/2*e1+s2*h/2*e2
        tl=(abot-o@a2)/(d@a2); tu=(ntop-(o-c)@N)/(d@N)
        verts[(s1,s2)]=(o+tl*d,o+tu*d)
    return Ps,d,e1,e2,verts
def report(Tuv,xs,w,h):
    Ps,d,e1,e2,v=arm(Tuv,xs,w,h)
    lows=[((lo-Ps)@ex+xs,(lo-Ps)@tg) for lo,up in v.values()]
    ups=[((up-c)@U,(up-c)@V) for lo,up in v.values()]
    cl=clearance(Ps,bp(Tuv[0],Tuv[1],2.25),w,h)
    print(Tuv,xs,w,h,'lowx',[round(l[0],1) for l in lows],'lowt',[round(l[1],1) for l in lows])
    print('   up u',[round(l[0],1) for l in ups],'v',[round(l[1],1) for l in ups],{k:round(float(x),1) for k,x in cl.items() if x<4})
if __name__=='__main__':
  for Tuv in [(-23,-17),(-24,-14)]:
    for xs in [-2,0,2]:
      report(Tuv,xs,10,10)
