import sys; sys.path.insert(0,'.')
from build123d import *
import numpy as np, submission as S
part=import_step('output.step')
A=import_step('fixed_A.step'); B=import_step('fixed_B.step')
vol=lambda x: 0 if x is None else x.volume
print('overlap A',vol(part&A),'B',vol(part&B))
for bid,b in S.BOLTS.items():
    p=np.array(b['position']);ax=np.array(b['axis'])
    hd=b['head_diameter'];hh=b['head_height']
    # find hole end: sample along axis
    # key shaft: cylinder from head top up 25mm
    g = S.A_GRIP if b['size']=='M5' else S.PLATE_T
    top=p+ax*(g+hh)
    pl=Plane(origin=S.v(p+ax*g),z_dir=S.v(ax))
    head=pl.location*Pos(0,0,(hh+1)/2)*Cylinder(hd/2+0.5,hh+1)
    shaft=Plane(origin=S.v(top),z_dir=S.v(ax)).location*Pos(0,0,12)*Cylinder(2.5,24)
    print(bid,'head ovl',round(vol(part&head),3),'shaft ovl',round(vol(part&shaft),3))
print('faces',len(part.faces()))
