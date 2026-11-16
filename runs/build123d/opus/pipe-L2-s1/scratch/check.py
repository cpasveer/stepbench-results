import json,numpy as np
from build123d import *
T=json.load(open('task.json'));P=T['poses'][0]
R=np.array(P['placements']['A']['rotation']); up=R[:,0]
s=import_step('output.step')
print('solids',len(s.solids()),'valid',s.is_valid, 'vol',s.volume)
for f in ['fixed_A.step','fixed_B.step']:
    fx=import_step(f); i=s & fx
    print(f,'overlap',i.volume if i else 0)
bb=s.bounding_box()
fs=s.faces()
print('faces',len(fs))
zs=[np.dot(np.array(tuple(v)),up) for v in s.vertices()]
zmin=min(zs); print('zmin',zmin)
for f in fs:
    c=f.center(); nrm=np.array(tuple(f.normal_at(c)))
    h=np.dot(np.array(tuple(c)),up)-zmin
    print(f"{f.geom_type!s:12} area={f.area:8.2f} cos_up={np.dot(nrm,up):+.3f} h={h:6.1f}")
es=s.edges(); print('min edge',min(e.length for e in es))
