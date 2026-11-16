import json,numpy as np
T=json.load(open('task.json'));P=T['poses'][0]
R=np.array(P['placements']['A']['rotation']);O=np.array(P['placements']['A']['origin'])
n=np.array(P['resolved']['mounting_faces'][0]['normal']);F=np.array(P['resolved']['mounting_faces'][0]['origin'])
RB=np.array(P['placements']['B']['rotation'])
loc=lambda p:R.T@(np.array(p)-O)
print('n local',R.T@n,'F local',loc(F))
print('B x,y local',R.T@RB[:,0],R.T@RB[:,1])
for b in P['resolved']['bolts']:
    print(b['id'],loc(b['position']).round(2),(R.T@np.array(b['axis'])).round(3))
