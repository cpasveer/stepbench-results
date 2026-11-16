from build123d import *
import sys
p=import_step('attempts/0005/output.step')
oA=Vector(95,-91,5.5); nA=Vector(0.7384553406258838,0.6743023875837234,0); tA=Vector(-nA.Y,nA.X,0)
for pt in [Vector(108.611,-105.906,0.4), Vector(112.838,-20.264,-64.9)]:
    print('point',pt)
    for i,f in enumerate(p.faces()):
        if f.distance_to(pt)<1.6:
            c=f.center(); r=c-oA
            try: nn=f.normal_at(c)
            except: nn=None
            print(' ',i,str(f.geom_type).split('.')[-1],round(f.area,1),'bbox',tuple(round(x,1) for x in f.bounding_box().min),tuple(round(x,1) for x in f.bounding_box().max))
