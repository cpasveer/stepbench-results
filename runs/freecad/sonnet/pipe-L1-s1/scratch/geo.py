import json,numpy as np
T=json.load(open('task.json'));P=T['poses'][0]
R=np.array(P['placements']['A']['rotation']);o=np.array(P['placements']['A']['origin'])
for b in P['resolved']['bolts']:
    p=np.array(b['position']);a=np.array(b['axis'])
    print(b['id'],np.round(R.T@(p-o),2),np.round(R.T@a,3))
Bo=np.array(P['placements']['B']['origin']);print('Bc local',np.round(R.T@(Bo-o),2))
RB=np.array(P['placements']['B']['rotation']);print(np.round(R.T@RB,3))
print('---B frame')
M=R.T@RB  # columns = B axes in pipe frame
Bc=R.T@(Bo-o)
for name,pt in [('axis0',np.zeros(3)),('top',np.array([0,0,34.])),('top+',np.array([0,0,42.]))]:
    print(name,np.round(M.T@(pt-Bc),1))
print('axis dir in B',np.round(M.T@np.array([1,0,0]),3))
print('pipe z-up in B',np.round(M.T@np.array([0,0,1]),3), 'y', np.round(M.T@np.array([0,1,0]),3))
