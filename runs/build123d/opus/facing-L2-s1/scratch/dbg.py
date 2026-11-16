import sys; sys.path.insert(0,'.')
import submission as S
from build123d import *
zlo, zhi = S.ZA + S.T_PLATE, S.ZB - S.T_PLATE
outer=S.kite()
tube = Pos(0,0,zlo)*extrude(outer, zhi - zlo)
part=(S.a_flange()+S.b_flange()+tube).clean()
print(part.is_valid, len(part.faces()))
for e in part.edges():
    c=e.center()
    if abs(c.Z-zlo)<1e-4 or abs(c.Z-zhi)<1e-4: print(round(c.X,2),round(c.Y,2),round(c.Z,2),round(e.length,2),e.geom_type)
print('---')
for f in part.faces():
    bb=f.bounding_box(); print(f.geom_type, [round(x,2) for x in (bb.min.X,bb.min.Y,bb.min.Z,bb.max.X,bb.max.Y,bb.max.Z)])
