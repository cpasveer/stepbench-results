import sys; sys.path.insert(0,'.')
import submission as s
from submission import *
a = s.pl2loc(s.P0[0], s.P0[1], 0.0); d=s.unit(s.S_END-a)
ex=np.array([1.0,0,0]); p1=s.unit(ex-d*(ex@d)); p2=np.cross(d,p1)
poly=[a+p1*6.5,a+p2*6.5,a-p1*6.5,a-p2*6.5]
seg=Solid.extrude(Face(Wire.make_polygon([V(q) for q in poly],close=True)),V(d*80))
print(seg.volume, seg.is_valid)
h=s.halfspace(s.BO+s.BN*3.0,s.BN,True)
print(h.volume,h.bounding_box())
r=Part(seg.wrapped)&h; print(r.volume)
r2=seg&h; print(r2.volume if r2 else None)
print(Part(seg.wrapped).volume)
