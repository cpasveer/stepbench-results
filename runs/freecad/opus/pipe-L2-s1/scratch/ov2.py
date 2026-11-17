import Part, FreeCAD as App, json
import numpy as np
A=Part.read('fixed_A.step')
T=json.load(open('task.json'));P=T['poses'][0]['placements']['A']
R=np.array(P['rotation']);O=np.array(P['origin'])
print(len(A.Faces))
for f in A.Faces:
    s=f.Surface
    bb=f.BoundBox
    pts=np.array([[p.x,p.y,p.z] for p in f.discretize(20)] if False else [[v.X,v.Y,v.Z] for v in f.Vertexes])
    print(type(s).__name__, round(f.Area,1), getattr(s,'Radius',None), ((np.array(bb.Center)-O)@R).round(1))
