import json, numpy as np
from build123d import *
t=json.load(open('task.json')); p=t['poses'][0]['placements']
RA=np.array(p['A']['rotation']); OA=np.array(p['A']['origin']);RB=np.array(p['B']['rotation']); OB=np.array(p['B']['origin'])
part=import_step('output.step')
for f in part.faces():
    c=np.array(tuple(f.center())); print(f.geom_type, round(f.area,2),'A-loc',np.round(RA.T@(c-OA),1),'B-loc',np.round(RB.T@(c-OB),1), len(f.edges()))
