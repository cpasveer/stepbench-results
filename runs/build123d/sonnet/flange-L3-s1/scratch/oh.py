import sys; sys.path.insert(0,'.')
import submission as S
from build123d import *
p=S.build()
print('vol',p.volume,len(p.solids()),p.is_valid,'conf',S.tool_conflicts(p))
tot=0
for i,f in enumerate(p.faces()):
    try:
        c=f.center(); n=f.normal_at(c)
    except Exception as e: continue
    dn=-n.Z
    if dn>0.7072:
        if abs(c.Z-S.ZBOT)<0.01 and f.geom_type==GeomType.PLANE: continue
        tot+=f.area
        print('overhang face',i,f.geom_type,round(f.area,1),'center',round(c.X,1),round(c.Y,1),round(c.Z,1),'ndown',round(dn,3))
print('overhang total',tot)
small=[(round(f.area,2),i) for i,f in enumerate(p.faces()) if f.area<3]
print('small faces',small)
print('short edges',[round(e.length,2) for e in p.edges() if e.length<0.6])
for i,f in enumerate(p.faces()):
    if f.area<3: print('SMALL',i,f.geom_type,f.area,f.center())
for e in p.edges():
    if e.length<0.6: print('SHORT',round(e.length,3),e.center())
