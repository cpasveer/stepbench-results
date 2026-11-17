import json,numpy as np
T=json.load(open('task.json'));P=T['poses'][0]['resolved']
for b in P['bolts']: print(b['id'],np.round(b['position'],1),np.round(b['axis'],2))
O=np.array([21,136.5,-7.]);d=np.array([.25,.59,.77]);d/=np.linalg.norm(d)
print(d)
for b in P['bolts'][:4]:
    p=np.array(b['position'])-O; print(b['id'],'axial',p@d,'radial',np.linalg.norm(p-(p@d)*d))
f=P['mounting_faces'][0];print(f)
