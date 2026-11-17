from ana import *
a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0]);tg=np.cross(a2,ex)
def arm(Tuv,xs,tt,w,h,ntop=1.5,abot=36.5,verbose=True):
    Ps=np.array([xs,0,0])+38*a2+tt*tg
    Tm=bp(Tuv[0],Tuv[1],2.25)
    d=Tm-Ps;d/=np.linalg.norm(d)
    e1=np.cross(d,N);e1/=np.linalg.norm(e1)  # horizontal-ish wrt plate: in plate plane
    e2=np.cross(d,e1)
    verts={}
    for s1 in(-1,1):
      for s2 in(-1,1):
        o=Ps+s1*w/2*e1+s2*h/2*e2
        # lower: plane a.(p)=abot
        tl=(abot-o@a2)/(d@a2)
        # upper: n coordinate: (p-c)@N=ntop
        tu=(ntop-(o-c)@N)/(d@N)
        verts[(s1,s2)]=(o+tl*d,o+tu*d)
    return Ps,d,e1,e2,verts
if __name__=='__main__':
  for Tuv in [(-23,-17),(-24,-15),(-22,-20)]:
    for xs in [-4,0]:
      Ps,d,e1,e2,v=arm(Tuv,xs,0,10,10)
      print(Tuv,xs,'d',d.round(2),'n.d',round(d@N,2),'a.d',round(d@a2,2))
      for k,(lo,up) in v.items():
        lo_t=(lo-Ps)@tg; up_uv=((up-c)@U,(up-c)@V)
        print(' ',k,'low x=%.1f tang=%.1f'%(lo[0],lo_t),'up uv=(%.1f,%.1f)'%up_uv)
      # clearance sampling using hull of verts: reuse clearance on centerline with w,h approx
      Tm=bp(Tuv[0],Tuv[1],2.25)
      cl=clearance(Ps,Tm,10,10)
      print(' clear',{k:round(float(x),1) for k,x in cl.items() if x<4})
