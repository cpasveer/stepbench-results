import json,numpy as np
T=json.load(open('task.json'));B={b['id']:b for b in T['poses'][0]['resolved']['bolts']}
ax=np.array([.25,.59,.77]);ax/=np.linalg.norm(ax);c=np.array([21,136.5,-7.])
for k,b in B.items():
    p=np.array(b['position']);a=np.array(b['axis'])
    print(k,p.round(1),'rad',np.linalg.norm((p-c)-ax*np.dot(p-c,ax)).round(2),'axial',np.dot(p-c,ax).round(1))
print(np.degrees(np.arccos(np.dot(ax,[.3885,.9214,0]))))
a1=np.array(B['A1']['axis']);a3=np.array(B['A3']['axis'])
t=a1+a3;t/=np.linalg.norm(t);s=np.cross(ax,t)
print('t',t,'s',s,np.degrees(np.arccos(a1@t)))
def loc(p): d=np.array(p)-c; return np.array([d@ax,d@t,d@s]).round(1)
for k in B: print(k,loc(B[k]['position']))
print('Bc',loc([19.5496,49.6857,52.2473]),'nB',np.array([ .3885,.9214,0])@ax,np.array([.3885,.9214,0])@t,np.array([.3885,.9214,0])@s)
