import sys; sys.path.insert(0,'.')
from submission import *
a=arm()
for s in a.solids() if hasattr(a,'solids') else []:
    print(len(s.vertices()))
vs=[np.array([v.X,v.Y,v.Z])-BO for v in a.vertices()]
for q in vs: print(round(q@BU,1),round(q@BV,1),round(q@BN,1))
