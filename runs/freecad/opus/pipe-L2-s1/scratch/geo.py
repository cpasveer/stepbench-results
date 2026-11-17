import numpy as np, json
T=json.load(open('task.json'))
pl=T['poses'][0]['placements']
RA=np.array(pl['A']['rotation']); OA=np.array(pl['A']['origin'])
RB=np.array(pl['B']['rotation']); OB=np.array(pl['B']['origin'])
ex,ey,ez=RA[:,0],RA[:,1],RA[:,2]
n=RB[:,2]
def loc(p): q=np.array(p)-OA; return q@ex,q@ey,q@ez
print('B center local',loc(OB),'n local',n@ex,n@ey,n@ez)
tB=6;PB=OB+tB*n
best=[]
for xs in np.arange(-10,15,1):
  for ph in np.arange(-10,40,2):
    r=41;f=np.radians(ph)
    S=OA+xs*ex+r*(np.sin(f)*ey+np.cos(f)*ez)
    c=S-PB; L=np.linalg.norm(c); c/=L
    rad=np.sin(f)*ey+np.cos(f)*ez
    a1=np.degrees(np.arccos(c@n)); a2=np.degrees(np.arccos(-c@rad))
    best.append((max(a1,a2),a1,a2,xs,ph,L))
best.sort()
for b in best[:10]: print(b)
print('---')
for b in best:
  if b[3] in (0,4,6,8) and b[4] in (0,4,8,12): print([round(float(v),1) for v in b])
print('===')
bu=RB[:,0];bv=RB[:,1]
for xs,ph in [(6,4),(4,8),(8,0),(0,8),(-6,10)]:
    f=np.radians(ph);S=OA+xs*ex+41*(np.sin(f)*ey+np.cos(f)*ez)
    c=S-PB;c/=np.linalg.norm(c)
    print(xs,ph,'lean in B (u,v):',round(c@bu,3),round(c@bv,3),'cn',round(c@n,3))
for b in T['poses'][0]['resolved']['bolts']:
    p=np.array(b['position']); 
    if b['id'][0]=='B': print(b['id'],round((p-OB)@bu,2),round((p-OB)@bv,2))
    else: print(b['id'],[round(v,2) for v in loc(p)])
print('pipe axis in B (u,v,n):',ex@bu,ex@bv,ex@n)
print('pipe top dir ez in B:',ez@bu,ez@bv,ez@n)
print('###')
for xs in [-8,-4,0,4,8]:
  for ph in [-26,-22,-18]:
    f=np.radians(ph);S=OA+xs*ex+41*(np.sin(f)*ey+np.cos(f)*ez);rad=np.sin(f)*ey+np.cos(f)*ez
    c=S-PB;L=np.linalg.norm(c);c/=L
    print(xs,ph,'aB',round(np.degrees(np.arccos(c@n)),1),'aA',round(np.degrees(np.arccos(-c@rad)),1),'uv',round(c@bu,2),round(c@bv,2),'L',round(L,1))
