import json, numpy as np
from build123d import *
t=json.load(open('task.json'))
p=t['poses'][0]
RA=np.array(p['placements']['A']['rotation']); OA=np.array(p['placements']['A']['origin'])
RB=np.array(p['placements']['B']['rotation']); OB=np.array(p['placements']['B']['origin'])
for f in 'AB':
    s=import_step(f'fixed_{f}.step'); bb=s.bounding_box(); print(f,bb.min,bb.max,s.volume, len(s.solids()), len(s.faces()))
print('B in A local', RA.T@(OB-OA)); print('B axes in A local\n', RA.T@RB)
for b in p['resolved']['bolts']:
    P=np.array(b['position']);print(b['id'],'A-local',np.round(RA.T@(P-OA),2),'B-local',np.round(RB.T@(P-OB),2))
# B solid extents in B local
s=import_step('fixed_B.step')
vs=np.array([RB.T@(np.array(tuple(v))-OB) for v in s.vertices()]);print('B local ext',vs.min(0),vs.max(0))
s=import_step('fixed_A.step')
vs=np.array([RA.T@(np.array(tuple(v))-OA) for v in s.vertices()]);print('A local ext',vs.min(0),vs.max(0))
