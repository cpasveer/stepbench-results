import json, numpy as np, itertools
from build123d import *
t=json.load(open('task.json')); bolts=t['poses'][0]['resolved']['bolts']
part=import_step('output.step'); A=import_step('fixed_A.step'); B=import_step('fixed_B.step')
print('solids',len(part.solids()),'faces',len(part.faces()),'valid',part.is_valid,'vol',part.volume)
print('min edge',min(e.length for e in part.edges()),'min face',min(f.area for f in part.faces()))
for f in part.faces(): print(f.geom_type, round(f.area,2))
print('ovl A',(part&A).volume if (part&A) else 0,'ovl B',(part&B).volume if (part&B) else 0)
obst=[part,A,B]
def perp(a):
    x=np.cross(a,[1,0,0]); 
    if np.linalg.norm(x)<.3: x=np.cross(a,[0,1,0])
    x/=np.linalg.norm(x); return x,np.cross(a,x)
def vol(s):
    try: return s.volume
    except Exception: return 0
for b in bolts:
    P=np.array(b['position']); a=np.array(b['axis']); m5=b['size']=='M5'
    hd,hh=b['head_diameter'],b['head_height']
    # grip: find via ray: sample along axis
    pl=Plane(origin=tuple(P),z_dir=tuple(a))
    # head clearance envelope
    grip=7.0 if m5 else 6.0
    env=Plane(origin=tuple(P+a*(grip+0.01)),z_dir=tuple(a))*Cylinder(hd/2+0.5,hh+1,align=(Align.CENTER,Align.CENTER,Align.MIN))
    hv=sum(vol(env&o) for o in obst)
    kd,short,long_=(4,25,70) if m5 else (3,20,63)
    x,y=perp(a); top=P+a*(grip+hh-0.6*kd)  # key tip in socket
    best=None
    for lean in (0,10,20,25):
        for laz in range(0,360,30):
            L=np.radians(lean); d=np.cos(L)*a+np.sin(L)*(np.cos(np.radians(laz))*x+np.sin(np.radians(laz))*y)
            leg=Plane(origin=tuple(top+d*(0.6*kd+0.3)),z_dir=tuple(d))*Cylinder(kd/2*1.16,short-0.6*kd-0.3,align=(Align.CENTER,Align.CENTER,Align.MIN))
            if sum(vol(leg&o) for o in obst)>1e-3: continue
            bend=top+d*(short-kd/2)
            dx,dy=perp(d); ok=[]
            for az in range(0,360,15):
                r=np.cos(np.radians(az))*dx+np.sin(np.radians(az))*dy
                arm=Plane(origin=tuple(bend),z_dir=tuple(r))*Cylinder(kd/2*1.16,long_,align=(Align.CENTER,Align.CENTER,Align.MIN))
                ok.append(sum(vol(arm&o) for o in obst)<1e-3)
            # longest run
            run=mx=0
            for v in ok+ok:
                run=run+1 if v else 0; mx=max(mx,run)
            mx=min(mx,24)
            if best is None or mx>best[0]: best=(mx,lean,laz)
        if best and best[0]>=6 and lean==0: break
    print(b['id'],'head-env overlap',round(hv,4),'key best free run (x15deg), lean, az:',best)
for b in bolts:
    P=np.array(b['position']); a=np.array(b['axis']); m5=b['size']=='M5'; grip=7.0 if m5 else 6.0
    for extra in (0.5,1.5,2.5):
        env=Plane(origin=tuple(P+a*(grip+0.01)),z_dir=tuple(a))*Cylinder(b['head_diameter']/2+extra,b['head_height']+1+extra,align=(Align.CENTER,Align.CENTER,Align.MIN))
        print(b['id'],extra,round(vol(env&part),3),end=' | ')
    print()
