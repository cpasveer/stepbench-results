import json,numpy as np
T=json.load(open('task.json'));P=T['poses'][0]['resolved']
B={b['id']:b for b in P['bolts']}
a1=np.array(B['A1']['axis']);a3=np.array(B['A3']['axis'])
O=np.array([21,136.5,-7.]);d=np.array([.25,.59,.77]);d/=np.linalg.norm(d)
nb=a1+a3;nb/=np.linalg.norm(nb);print('nb',nb,'angle between',np.degrees(np.arccos(a1@a3)), 'nb.d',nb@d)
L=O+34*nb;C=np.array(P['mounting_faces'][0]['origin']);n=np.array(P['mounting_faces'][0]['normal'])
print('L',L,'C',C,'dist',np.linalg.norm(L-C),'dir',(L-C)/np.linalg.norm(L-C),'n',n)
# B frame
print('n.d',n@d,'n.nb',n@nb)
