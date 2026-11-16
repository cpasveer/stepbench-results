from build123d import *
p=import_step('attempts/0004/output.step')
f=p.faces()[39]
for v in f.vertices(): print(round(v.X,2),round(v.Y,2),round(v.Z,2))
print('--- face 12 top')
f=p.faces()[12]
for v in f.vertices(): print(round(v.X,2),round(v.Y,2),round(v.Z,2))
