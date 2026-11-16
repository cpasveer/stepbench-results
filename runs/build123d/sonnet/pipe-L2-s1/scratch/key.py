import sys; sys.path.insert(0,'.')
from submission import *
import math
part=to_world(build())
fa=import_step("fixed_A.step"); fb=import_step("fixed_B.step")
obst=Compound([part,fa,fb]) if False else None
def slab_sector(p,a,t0,hz,radius,thick,ang=60):
    a=unit(a); r=np.cross(a,[0.3,0.5,0.8]); r=unit(r); s=np.cross(a,r)
    pts=[p+a*(hz-thick/2)]
    poly=[V(p+a*(hz-thick/2))]
    n=12
    for i in range(n+1):
        th=math.radians(t0+ang*i/n)
        poly.append(V(p+a*(hz-thick/2)+ (r*math.cos(th)+s*math.sin(th))*radius))
    f=Face(Wire.make_polygon(poly,close=True))
    return Solid.extrude(f,V(a*thick))
def post(p,a,h,rad):
    pl=Plane(origin=V(p),z_dir=V(unit(a)))
    return pl*Cylinder(rad,h,align=(Align.CENTER,Align.CENTER,Align.MIN))
for k,b in BOLTS.items():
    p=np.array(b["position"]); a=np.array(b["axis"]); 
    m5=b["size"]=="M5"
    g=6.0; hh=b["head_height"]; he=hh+(20.5 if m5 else 16.5)+g
    ok=[]
    for t0 in range(0,360,15):
        sl=slab_sector(p,a,t0,he,70 if m5 else 63,4 if m5 else 3)
        v=0
        for o in (part,fa,fb):
            r=sl&o
            v+=(r.volume if r is not None else 0)
        if v<0.05: ok.append(t0)
    # also shaft post
    pv=sum(((lambda r: r.volume if r is not None else 0)(post(p+unit(a)*(g+hh-2.5),a,he-hh+2.5,2.2)&o)) for o in (part,fa,fb))
    print(k,"free start angles",ok,"post vol",round(pv,2))
