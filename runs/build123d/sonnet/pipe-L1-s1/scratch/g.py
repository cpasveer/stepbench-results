import json,numpy as np
T=json.load(open('task.json'));B={b['id']:b for b in T['poses'][0]['resolved']['bolts']}
P={k:np.array(v['position']) for k,v in B.items()}
a0=np.array([21,136.5,-7.]);ad=np.array([.25,.59,.77]);ad/=np.linalg.norm(ad)
for k in P:
    r=P[k]-a0;print(k,np.linalg.norm(r-r@ad*ad), (P[k]-a0)@ad)
print((P['A1']+P['A2'])/2,(P['A3']+P['A4'])/2,(P['B1']+P['B2']+P['B3']+P['B4'])/4)
n1=np.array(B['A1']['axis']);n2=np.array(B['A3']['axis']);m=n1+n2;m/=np.linalg.norm(m)
C0=a0+34*m;print('m',m,'C0',C0)
nb=np.array(B['B1']['axis']);cb=np.array([19.5496,49.6857,52.2473])
print('C0-cb',C0-cb,'dot nb',(C0-cb)@nb)
