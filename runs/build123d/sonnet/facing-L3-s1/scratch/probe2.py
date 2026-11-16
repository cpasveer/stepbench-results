from build123d import *
s=import_step('output.step')
for f in s.faces():
    c=f.center()
    if abs(c.Z+35)<0.01 or f.geom_type==GeomType.PLANE and abs(f.normal_at().Z)>0.99 and -40<c.Z<-26:
        print(f.geom_type,round(f.area,1),c, f.normal_at())
