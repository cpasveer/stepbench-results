import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
S.WEB_PAD_THETA = -45; S.WEB_PAD_S = 0; S.WEB_PAD_R = 38.0; S.WEB_PLATE_DT = 6.0
part = S.build()
P0 = Vector(*S.PIPE_PLACE["origin"])
for f in part.faces():
    n = f.normal_at(f.center())
    if n.Z < -0.72 and f.area > 1.0:
        c = f.center()
        print(round(f.area,1), f.geom_type.name, tuple(round(x,2) for x in (n.X,n.Y,n.Z)), tuple(round(x,1) for x in (c.X,c.Y,c.Z)))
