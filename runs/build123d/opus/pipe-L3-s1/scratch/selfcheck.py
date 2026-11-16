import json, numpy as np
from build123d import *
T=json.load(open('task.json'));R=T['poses'][0]['resolved']
P=import_step('output.step'); A=import_step('fixed_A.step'); B=import_step('fixed_B.step')
print('solids',len(P.solids()),'valid',P.is_valid,'vol',round(P.volume,1),'faces',len(P.faces()))
print('bbox',P.bounding_box())
for nm,F in (('A',A),('B',B)):
    print('overlap',nm, round((P&F).volume,4))
for b in R['bolts']:
    p=np.array(b['position']);a=np.array(b['axis']);hd=b['head_diameter'];hh=b['head_height']
    # find grip: walk along axis to find where material ends
    pts=[];
    g=None
    for t in np.arange(0.05,30,0.05):
        q=p+a*(t)+ np.cross(a,[0.3,0.2,0.9])/np.linalg.norm(np.cross(a,[0.3,0.2,0.9]))*(b['clearance_diameter_range'][0]/2+0.4)
        if P.is_inside(Vector(*q)): g=t
    pl=Plane(origin=tuple(p+a*g),z_dir=tuple(a))
    head=pl.location*Cylinder(hd/2+0.5,hh+1,align=(Align.CENTER,Align.CENTER,Align.MIN))
    pl2=Plane(origin=tuple(p+a*(g+hh)),z_dir=tuple(a))
    tool=pl2.location*Cylinder(hd/2,30,align=(Align.CENTER,Align.CENTER,Align.MIN))
    print(b['id'],'grip~',round(g,2),'head∩P',round((head&P).volume,3),'tool∩P',round((tool&P).volume,3),'tool∩A',round((tool&A).volume,3),'tool∩B',round((tool&B).volume,3))
fa=sorted(P.faces(),key=lambda f:f.area)[:6]
print('smallest faces',[(f.geom_type.name,round(f.area,2)) for f in fa])
print('shortest edges',sorted(round(e.length,3) for e in P.edges())[:6])
