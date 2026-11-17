import sys,numpy as np,math
from build123d import *
part=import_step(sys.argv[1])
from collections import defaultdict
emap=defaultdict(list);ekeys={}
for f in part.faces():
    for e in f.edges():
        c=e.center();k=(round(c.X,2),round(c.Y,2),round(c.Z,2),round(e.length,2))
        emap[k].append(f);ekeys[k]=e
solid=part.solids()[0]
res=[]
for k,fs in emap.items():
    e=ekeys[k]
    if len(fs)!=2: continue
    p=e.position_at(0.5)
    n1=fs[0].normal_at(p);n2=fs[1].normal_at(p)
    ang=math.degrees(math.acos(max(-1,min(1,n1.dot(n2)))))
    tg=e.tangent_at(0.5).normalized()
    def inward(f):
        c=f.center()-p
        c=c-tg*c.dot(tg)
        return c.normalized()
    d1=inward(fs[0]);conc=d1.dot(n2)<-1e-3
    air=180-ang
    res.append((air if conc else None,ang,conc,e.length,(p.X,p.Y,p.Z),fs[0].geom_type,fs[1].geom_type))
for r in sorted([r for r in res if r[2]],key=lambda r:r[0])[:8]:
    print('concave air angle',round(r[0],1),'len',round(r[3],1),np.round(r[4],1),r[5],r[6])
cnt=0
for k,fs in emap.items():
    if len(fs)==2 and fs[0].geom_type==fs[1].geom_type==GeomType.PLANE:
        p=ekeys[k].position_at(.5)
        if fs[0].normal_at(p).dot(fs[1].normal_at(p))>0.99999: cnt+=1
print('coplanar adjacent pairs',cnt)
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
c2=0
for k,fs in emap.items():
    if len(fs)==2 and fs[0].geom_type==fs[1].geom_type==GeomType.CYLINDER:
        a=[BRepAdaptor_Surface(f.wrapped).Cylinder() for f in fs]
        r=[x.Radius() for x in a];ax=[x.Axis().Direction() for x in a];lo=[x.Location() for x in a]
        d=ax[0].XYZ().Dot(ax[1].XYZ())
        if abs(r[0]-r[1])<1e-6 and abs(abs(d)-1)<1e-9:
            # same axis line?
            v=lo[0].XYZ().Subtracted(lo[1].XYZ());cr=v.Crossed(ax[0].XYZ()).Modulus()
            if cr<1e-6: c2+=1;print('unmerged cyl r',r[0],ekeys[k].center())
print('cyl pairs',c2)
print('--- edges with normal angle >100 (sharp):')
for r in sorted(res,key=lambda r:-r[1])[:6]:
    print(round(r[1],1),'conc',r[2],'len',round(r[3],1),np.round(r[4],1),r[5],r[6])
print('--- detail')
for k,fs in emap.items():
    e=ekeys[k]
    if len(fs)==2 and abs(e.center().X+1.8)<0.6 and abs(e.center().Y-68.4)<0.6:
        for f in fs: print(f.geom_type,round(f.area,1),f.normal_at(e.position_at(.5)),f.center())
