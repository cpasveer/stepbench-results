import numpy as np, json
exec(open('scratch/geo.py').read().split("print('B center")[0])
rad=lambda p: np.sin(np.radians(p))*ey+np.cos(np.radians(p))*ez
a1=rad(-35);a3=rad(35)
crit=[('Bc',-n),('Bs',n),('a1',a1),('a3',a3)]+[('sad%d'%p,-rad(p)) for p in range(-46,47,2)]
best=[]
rng=np.random.default_rng(0)
D=rng.normal(size=(200000,3));D/=np.linalg.norm(D,axis=1)[:,None]
N=np.array([c[1] for c in crit])
M=(D@N.T).max(axis=1)
i=np.argsort(M)[:5]
for k in i: 
    d=D[k];print(round(M[k],3),'d local',np.round([d@ex,d@ey,d@ez],3),'d.n',round(d@n,3))
# without Bc and Bs as constraints (one on bed)
for skip in ['Bc']:
    N2=np.array([c[1] for c in crit if c[0]!=skip])
    print(skip,'d=-n value',(N2@(-n)).max())
