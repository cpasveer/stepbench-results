from build123d import *
p=import_step('attempts/0004/output.step')
pt=Vector(95.876,-91.795,23.6)
oA=Vector(95,-91,5.5); nA=Vector(0.7384553406258838,0.6743023875837234,0); tA=Vector(-nA.Y,nA.X,0)
for i,f in enumerate(p.faces()):
    if f.distance_to(pt)<2.5:
        c=f.center(); r=c-oA
        try: nn=f.normal_at(c)
        except: nn=None
        print(i,f.geom_type,round(f.area,1),'u',round(r.dot(tA),1),'n',round(r.dot(nA),1),'w',round(r.Z,1),nn)
