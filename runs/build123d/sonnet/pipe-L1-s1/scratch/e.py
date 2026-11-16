from build123d import import_step
import numpy as np
o=import_step('output.step')
for e in o.edges():
    fs=[f for f in o.faces() if any(e.is_same(x) for x in f.edges())]
    if len(fs)==2:
        p=e.center()
        n=[f.normal_at(p) for f in fs]
        ang=np.degrees(np.arccos(np.clip(n[0].dot(n[1]),-1,1)))
        if ang<60 or ang>120: print(round(ang,1),[f.geom_type.name for f in fs],round(e.length,1),p)
