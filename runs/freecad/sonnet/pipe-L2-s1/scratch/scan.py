import json, numpy as np
T=json.load(open('task.json'))
P=T['poses'][0]
R=np.array(P['placements']['A']['rotation']); xa,ya,za=R[:,0],R[:,1],R[:,2]
n=np.array(P['resolved']['mounting_faces'][0]['normal'])
Rb=np.array(P['placements']['B']['rotation']); e1=Rb[:,0]; e2=Rb[:,1]
B={b['id']:np.array(b['axis']) for b in P['resolved']['bolts']}
a1,a3=B['A1'],B['A3']
PA=np.array(P['placements']['A']['origin'])+za*35.5
PB=np.array(P['placements']['B']['origin'])+n*5
r=(PB-PA);r/=np.linalg.norm(r)
print('r',r,'xa',xa,'ya',ya,'za',za,'n',n,'e1',e1,'e2',e2)
best=[]
rng=np.random.default_rng(0)
U=rng.normal(size=(400000,3));U/=np.linalg.norm(U,axis=1)[:,None]
c=0.68
def score(u):
    s=[]
    # plate faces
    for v in (n,-n,e1,-e1,e2,-e2): s.append(v@u)
    s=[min(x,1) for x in s]
    plate=min(s)
    seats=min(a1@u,a3@u)
    rod=abs(r@u)-0.72
    arc=min(-(np.cos(p)*za+np.sin(p)*ya)@u for p in np.radians(np.linspace(-52,52,9)))
    outer=min((np.cos(p)*za+np.sin(p)*ya)@u for p in np.radians([-35,0,35]))
    return plate,seats,rod,arc,outer
for u in U:
    p,s,ro,ar,ou=score(u)
    m=min(p+c,s+c,ro,ar+c,ou+c)
    best.append((m,u,(p,s,ro,ar,ou)))
best.sort(key=lambda t:-t[0])
for m,u,sc in best[:5]: print(round(m,3),u.round(3),np.round(sc,3))
