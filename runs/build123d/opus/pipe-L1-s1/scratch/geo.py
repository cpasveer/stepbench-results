import json,numpy as np
T=json.load(open('task.json'));P=T['poses'][0]['placements']
RA=np.array(P['A']['rotation']);OA=np.array(P['A']['origin'])
RB=np.array(P['B']['rotation']);OB=np.array(P['B']['origin'])
top=OA+34*RA[:,2]
print('top',top)
v=top-OB
print('B-local of saddle top', RB.T@v)
print('A-local of B center', RA.T@(OB-OA))
print('B normal in A local', RA.T@RB[:,2])
print('A axis in B local', RB.T@RA[:,0])
print('OA in B', RB.T@(OA-OB))
print('B-y in A', RA.T@RB[:,1], 'B-x in A', RA.T@RB[:,0])
for b in T['poses'][0]['resolved']['bolts']:
    print(b['id'], np.round(RB.T@(np.array(b['position'])-OB),2), np.round(RB.T@np.array(b['axis']),3))
