import json
from build123d import import_step, Cylinder, Plane, Vector, Align
o=import_step('output.step'); print(len(o.solids()), o.volume)
bb=o.bounding_box(); print(bb.min,bb.max)
for n in 'AB':
    f=import_step(f'fixed_{n}.step'); print(n,'overlap',((o&f).volume if (o&f) else 0))
T=json.load(open('task.json'))['poses'][0]['resolved']['bolts']
for b in T:
    hd,hh=b['head_diameter'],b['head_height']
    ax=Vector(*b['axis']);p=Vector(*b['position'])
    c=Cylinder(hd/2+.5,hh+1+0.0,align=(Align.CENTER,Align.CENTER,Align.MIN))
    c=Plane(origin=p+ax*7,z_dir=ax).location*c  # starts at top of 7mm pad
    print(b['id'],'head keepout overlap',round(((o&c).volume if (o&c) else 0),3))
