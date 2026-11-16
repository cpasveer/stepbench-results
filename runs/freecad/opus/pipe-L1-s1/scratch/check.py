import json,math,os,sys
import FreeCAD as App, Part
V=App.Vector
HERE=os.getcwd()
T=json.load(open('task.json'));B=T['poses'][0]['resolved']['bolts']
part=Part.read('output.step')
print('solids',len(part.Solids),'valid',part.isValid(),'faces',len(part.Faces),'vol',round(part.Volume))
for f in ['fixed_A.step','fixed_B.step']:
    print(f,'overlap',round(part.common(Part.read(f)).Volume,4))
fixed=Part.read('fixed_A.step').fuse(Part.read('fixed_B.step'))
obst=part.fuse(fixed)
KEY={'M4':(3.0,20,63,2.0),'M5':(4.0,25,70,2.5)}
def perp(a):
    t=V(1,0,0) if abs(a.x)<0.9 else V(0,1,0)
    u=a.cross(t);u.normalize();w=a.cross(u);return u,w
for b in B:
    p=V(*b['position']);a=V(*b['axis']);
    # find grip: hole end along axis
    hd=b['head_diameter'];hh=b['head_height']
    # find seat: first point along axis outside material
    g=None
    for i in range(1,300):
        t=i*0.1; q=p+a*t
        u,w=perp(a)
        if not part.isInside(q+u*(hd/2-0.3),1e-6,True): g=t;break
    head=Part.makeCylinder(hd/2+0.5,hh+1,p+a*g,a)
    hv=head.common(part).Volume
    kd,ls,ll,sd=KEY[b['size']]
    top=p+a*(g+hh)
    res=[]
    for lean in (0,10,20,25):
      u,w=perp(a)
      okang=[]
      for k in range(72):
        th=math.radians(k*5)
        dirl=u*math.cos(th)+w*math.sin(th)
        ax=a*math.cos(math.radians(lean))-dirl*math.sin(math.radians(lean))  # lean away from long leg
        bot=top-a*sd
        sl=Part.makeCylinder(kd/2,ls-kd/2,bot,ax)
        elb=bot+ax*(ls-kd/2)
        ld=dirl*math.cos(math.radians(lean))+a*math.sin(math.radians(lean))
        ll_=Part.makeCylinder(kd/2,ll,elb-ld*(kd/2),ld)
        key=sl.fuse(ll_)
        # exclude the head itself: only collide with part+fixed outside head cylinder
        c=key.common(obst).Volume
        okang.append(c<0.01)
      # longest circular run
      best=0;run=0
      for x in okang+okang:
        run=run+1 if x else 0;best=max(best,run)
      res.append((lean,min(best,72)*5))
    print(b['id'],'grip',round(g,2),'headclash',round(hv,3),'keyswing',res)
