import Part, FreeCAD as App, json, math, sys
V=App.Vector
T=json.load(open('task.json')); P=T['poses'][0]
part=Part.read('output.step')
A=Part.read('fixed_A.step'); B=Part.read('fixed_B.step')
print('solids',len(part.Solids),'valid',part.isValid(),'vol',round(part.Volume,1),'faces',len(part.Faces))
for nm,f in (('A',A),('B',B)):
    print('overlap',nm,round(part.common(f).Volume,4))
RA=P['placements']['A']['rotation']; EX=V(RA[0][0],RA[1][0],RA[2][0])
down=EX*-1 
KEY={'M5':(4.0,25.0,70.0),'M4':(3.0,20.0,63.0)}
obst=part.fuse(A).fuse(B)
def perp(a):
    t=V(1,0,0) if abs(a.x)<0.9 else V(0,1,0)
    e1=a.cross(t).normalize(); e2=a.cross(e1); return e1,e2
for b in P['resolved']['bolts']:
    p=V(*b['position']);a=V(*b['axis']).normalize()
    # find grip: distance along axis to where part ends (exit of hole)
    # head envelope
    # seat height: probe along axis
    lo,hi=0,40
    e1,e2=perp(a)
    # find seat: largest s where point at radius (hole+0.5) is inside part
    rr=(b['head_diameter']/2+ 3.0)/2
    s=0
    for k in range(400):
        q=p+a*(k*0.1)+e1*rr
        if part.isInside(q,0.01,True): s=k*0.1
    grip=s
    hd=b['head_diameter']/2+0.5; hh=b['head_height']+1
    head=Part.makeCylinder(hd,hh,p+a*(grip+0.02),a)
    hv=head.common(obst).Volume
    sz,ls,ll=KEY[b['size']]
    rk=sz/2/math.cos(math.radians(30))
    top=p+a*(grip+b['head_height'])
    free={}
    for lean in (0,12,25):
      for beta in (180,90,-90,0):
        okang=[]
        for ps in range(0,360,10):
            ps_r=math.radians(ps)
            e=e1*math.cos(ps_r)+e2*math.sin(ps_r)
            br=math.radians(ps+beta); el=e1*math.cos(br)+e2*math.sin(br)
            k=(a*math.cos(math.radians(lean))+el*math.sin(math.radians(lean))).normalize()
            l=(e-k*e.dot(k)).normalize()
            leg1=Part.makeCylinder(rk,ls-sz/2,top,k)
            c=top+k*(ls-sz/2)
            leg2=Part.makeCylinder(rk,ll-sz/2,c,l)
            key=leg1.fuse(leg2)
            okang.append(key.common(obst).Volume<0.01)
        # longest run
        best=0;run=0
        for v in okang+okang:
            run=run+1 if v else 0; best=max(best,run)
        free[(lean,beta)]=min(best,36)
    bestw=max(free.values())
    print(b['id'],'grip',round(grip,2),'headclash',round(hv,3),'max free run (x10deg)',bestw, 'by',[k for k,v in free.items() if v==bestw][:3])
# overhang
import numpy as np
bad=0;tot=0
zmin=min(v.Point.dot(down*-1) for v in part.Vertexes)
for f in part.Faces:
    u0,u1,v0,v1=f.ParameterRange
    ar=f.Area; s=0;cnt=0
    for i in range(6):
      for j in range(6):
        uu=u0+(u1-u0)*(i+0.5)/6; vv=v0+(v1-v0)*(j+0.5)/6
        pt=f.valueAt(uu,vv)
        if not f.isInside(pt,1e-3,True): continue
        nn=f.normalAt(uu,vv)
        cnt+=1
        if nn.dot(down)>0.7071 and pt.dot(down*-1)-zmin>0.2: s+=1
    if cnt and s:
        print('overhang face',f.Surface.__class__.__name__,round(ar,1),'frac',round(s/cnt,2))
