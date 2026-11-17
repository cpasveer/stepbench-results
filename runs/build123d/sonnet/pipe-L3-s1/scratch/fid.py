import sys
from build123d import *
part=import_step('output.step')
fs=part.faces()
for i in map(int,sys.argv[1:]):
    f=fs[i];n=f.normal_at(f.center())
    print(i,f.geom_type,round(f.area,1),f.center(),'n',round(n.X,2),round(n.Y,2),round(n.Z,2))
