import pathlib
from build123d import *
p = pathlib.Path(__file__).parent.parent / "output.step"
s = import_step(str(p)).solids()[0]
for f in s.faces():
    bb=f.bounding_box()
    if bb.min.Z <= -63.9 and bb.max.Z <= -63.9: continue   # bed
    worst=0.0; wpt=None
    for i in range(7):
        for j in range(7):
            try:
                uv=(i/6,j/6)
                pt=f.position_at(*uv); n=f.normal_at(*uv)
            except Exception: continue
            if n.Z < worst: worst=n.Z; wpt=pt
    if worst < -0.5:
        print("area %.1f  min nz %.2f  at (%.1f,%.1f,%.1f)  type %s  z[%.1f,%.1f]"%(f.area, worst, wpt.X,wpt.Y,wpt.Z, f.geom_type.name, bb.min.Z, bb.max.Z))
