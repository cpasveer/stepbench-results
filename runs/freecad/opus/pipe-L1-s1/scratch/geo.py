import json,numpy as np
T=json.load(open('task.json'));P=T['poses'][0]
for k in 'AB':
    o=np.array(P['placements'][k]['origin']);R=np.array(P['placements'][k]['rotation'])
    print(k,o,R.T)
oA=np.array(P['placements']['A']['origin']);RA=np.array(P['placements']['A']['rotation'])
for b in P['resolved']['bolts']:
    p=np.array(b['position']);a=np.array(b['axis'])
    print(b['id'],'localA',np.round(RA.T@(p-oA),2),np.round(RA.T@a,3))
f=P['resolved']['mounting_faces'][0]
print('Bface localA',RA.T@(np.array(f['origin'])-oA),RA.T@np.array(f['normal']))
oB=np.array(P['placements']['B']['origin']);RB=np.array(P['placements']['B']['rotation'])
print('B axes in A',RA.T@RB)
