import sys; sys.path.insert(0,'.')
from submission import *
part=build()
for i,f in enumerate(part.faces()):
    c=f.center(); n=None
    try: nn=f.normal_at(); n=np.round([nn.X,nn.Y,nn.Z],2)
    except Exception: pass
    cq=np.array([c.X,c.Y,c.Z])
    print(i,f.geom_type.name,round(f.area,1),"center plate uvn",np.round([(cq-BO)@BU,(cq-BO)@BV,(cq-BO)@BN],1),n)
