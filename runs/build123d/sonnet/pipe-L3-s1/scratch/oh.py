import numpy as np
from build123d import *
part=import_step('output.step')
for w in [(-0.4,0.82,0.42),(0.64,0.48,-0.6)]:
    w=np.array(w);w/=np.linalg.norm(w);print(w)
    for i,f in enumerate(part.faces()):
        n=f.normal_at().to_tuple() if hasattr(f.normal_at(),'to_tuple') else (f.normal_at().X,f.normal_at().Y,f.normal_at().Z)
        c=np.dot(n,w)
        if c<-0.7071: print(i,f.geom_type,round(f.area,1),np.round(c,2),np.round(f.center().to_tuple() if hasattr(f.center(),'to_tuple') else (f.center().X,f.center().Y,f.center().Z),1))
