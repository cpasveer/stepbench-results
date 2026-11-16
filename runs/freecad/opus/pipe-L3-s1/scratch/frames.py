import json, numpy as np
T=json.load(open('task.json'))
pl=T['poses'][0]['placements']
RA=np.array(pl['A']['rotation']); CA=np.array(pl['A']['origin'])
RB=np.array(pl['B']['rotation']); CB=np.array(pl['B']['origin'])
loc=lambda p: RA.T@(np.array(p)-CA)
locv=lambda v: RA.T@np.array(v)
for b in T['poses'][0]['resolved']['bolts']:
    print(b['id'], np.round(loc(b['position']),3), np.round(locv(b['axis']),4))
print('Bc', loc(CB)); 
for i,n in enumerate('UVN'): print(n, locv(RB[:,i]))
