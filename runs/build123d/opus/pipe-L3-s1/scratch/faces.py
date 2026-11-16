import numpy as np
from build123d import *
P=import_step('output.step')
for i,f in enumerate(P.faces()):
    c=f.center()
    nrm = f.normal_at(c) if f.geom_type==GeomType.PLANE else None
    print(i,f.geom_type.name,round(f.area,1),np.round(tuple(c),1), np.round(tuple(nrm),3) if nrm else '', len(f.edges()))
