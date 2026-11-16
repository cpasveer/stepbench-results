from build123d import *
p=import_step('attempts/0002/output.step')
oA=Vector(95,-91,5.5); nA=Vector(0.7384553406258838,0.6743023875837234,0); tA=Vector(-nA.Y,nA.X,0)
for i,f in enumerate(p.faces()):
    if f.area<80:
        c=f.center(); r=c-oA
        print(i,f.geom_type,round(f.area,2),'u',round(r.dot(tA),2),'n',round(r.dot(nA),2),'w',round(r.Z,2))
