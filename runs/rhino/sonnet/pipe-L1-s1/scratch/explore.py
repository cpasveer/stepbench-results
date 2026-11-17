import json, numpy as np
T=json.load(open('task.json'));B={b['id']:b for b in T['poses'][0]['resolved']['bolts']}
O=np.array([21,136.5,-7.]);d=np.array([.25,.59,.77]);d/=np.linalg.norm(d)
def rad(p):
    v=np.array(p)-O;v-=v.dot(d)*d;return np.linalg.norm(v),v/np.linalg.norm(v)
for k in ['A1','A2','A3','A4']:
    r,n=rad(B[k]['position']);print(k,r,n,np.array(B[k]['axis']),(np.array(B[k]['position'])-O).dot(d))
n1=np.array(B['A1']['axis']);n2=np.array(B['A3']['axis'])
m=n1+n2;m/=np.linalg.norm(m);print('m',m,m.dot(d),np.degrees(np.arccos(n1.dot(n2)))/2)
CB=np.array([19.549595153015062,49.68566626427389,52.2472790015625])
N=np.array([0.3885173161164798,0.9214414224885092,0]);Z=np.array([0,0,1.]);Tt=np.cross(N,Z)
E=CB+(-11.96)*Z+2.95*N
for a0 in [-10,0,10,20]:
    P0=O+a0*d+41.5*m;s=P0-E;L=np.linalg.norm(s);s/=L
    print(a0,L,'ang to plane',np.degrees(np.arcsin(s.dot(N))), 'bframe',s.dot(Tt),s.dot(Z),s.dot(N))
