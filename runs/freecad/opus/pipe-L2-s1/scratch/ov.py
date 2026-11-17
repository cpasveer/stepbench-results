import Part, FreeCAD as App, json
part=Part.read('output.step'); A=Part.read('fixed_A.step')
c=part.common(A)
for s in c.Solids: print(round(s.Volume,2), s.BoundBox.Center)
import sys; sys.path.insert(0,'.')
T=json.load(open('task.json'));P=T['poses'][0]['placements']['A']
import numpy as np
R=np.array(P['rotation']);O=np.array(P['origin'])
for s in c.Solids:
    p=np.array(s.BoundBox.Center); l=(p-O)@R; print('local',l.round(2),'r',np.hypot(l[1],l[2]).round(2), 'phi',np.degrees(np.arctan2(l[1],l[2])).round(1))
