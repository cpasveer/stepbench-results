import sys; sys.path.insert(0,".")
import submission as S
from build123d import *
p=S.build(); inv=Location((S.ORG.X,S.ORG.Y,0),(0,0,S.ANG)).inverse()
q=inv*p
for f in sorted(q.faces(),key=lambda f:f.area)[:4]: print("F",round(f.area,4),f.geom_type,f.center())
for e in sorted(q.edges(),key=lambda e:e.length)[:8]: print("E",round(e.length,4),e.geom_type,e.center())
print(S.to_design(S.PB["origin"]))
