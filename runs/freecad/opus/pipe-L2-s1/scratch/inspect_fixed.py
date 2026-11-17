import Part, FreeCAD as App
import numpy as np, json
T=json.load(open('task.json'));pl=T['poses'][0]['placements']
for nm in 'AB':
    s=Part.read('fixed_%s.step'%nm)
    print(nm,len(s.Solids),s.BoundBox,s.Volume)
    R=np.array(pl[nm]['rotation']);O=np.array(pl[nm]['origin'])
    pts=np.array([[v.X,v.Y,v.Z] for v in s.Vertexes])
    loc=(pts-O)@R
    print(' local bbox',loc.min(0).round(2),loc.max(0).round(2))
    for i,sol in enumerate(s.Solids):
        p=np.array([[v.X,v.Y,v.Z] for v in sol.Vertexes]); l=(p-O)@R
        print('  solid',i,round(sol.Volume,1),l.min(0).round(1),l.max(0).round(1))
