from build123d import *
s=import_step('output.step')
for f in s.faces():
    n=f.normal_at()
    if n.Z<-0.5 and f.center().Z>-93.9:
        print(f.geom_type,round(f.area,1),f.center(),n)
