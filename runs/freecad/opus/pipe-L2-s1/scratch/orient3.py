import numpy as np, json
exec(open('scratch/geo.py').read().split("print('B center")[0])
bu=RB[:,0];bv=RB[:,1]
rad=lambda p: np.sin(np.radians(p))*ey+np.cos(np.radians(p))*ez
a1=rad(-35);a3=rad(35)
crit=[-n,n,a1,a3]+[-rad(p) for p in range(-50,51,2)]
N=np.array(crit)
sad=np.array([OA+x*ex+r*rad(p) for x in (-27,27) for r in (34,42) for p in range(-50,51,5)])
pl=np.array([OB+su*bu+sv*bv+t*n for su in (-17,17) for sv in (-17,17) for t in (0,6)])
rng=np.random.default_rng(1)
D=rng.normal(size=(400000,3));D/=np.linalg.norm(D,axis=1)[:,None]
M=(D@N.T).max(axis=1)
smin=(sad@D.T).max(axis=0); pmin=(pl@D.T).max(axis=0)   # lowest = max along d
diff=pmin-smin   # >0: plate lower than saddle
for lim in (0.6,0.66,0.7):
  ok=M<lim
  k=np.argmax(np.where(ok,diff,-1e9)); d=D[k]
  print(lim,'max plate-lower-than-saddle',round(diff[k],1),'M',round(M[k],3),'d',np.round(d,3),'d.ex',round(d@ex,2),'d.bu',round(d@bu,2),'d.bv',round(d@bv,2),'d.n',round(d@n,2))
E=np.array([bu,-bu,bv,-bv])
ME=(D@E.T).max(axis=1)
ok=(M<0.66)&(np.abs(diff)<2)&(ME<0.7)
print('count',ok.sum())
idx=np.where(ok)[0]
# prefer d with small M overall
for k in idx[np.argsort(np.maximum(M[idx],ME[idx]))][:8]:
    d=D[k]; print('M',round(M[k],3),'ME',round(ME[k],3),'diff',round(diff[k],1),'d',np.round(d,3),'ex',round(d@ex,2),'bu',round(d@bu,2),'bv',round(d@bv,2),'n',round(d@n,2))
print('--- bv~0')
ok=(M<0.7)&(np.abs(D@bv)<0.02)
idx=np.where(ok)[0]
for k in idx[np.argsort(-diff[idx])][:5]:
    d=D[k]; print('M',round(M[k],3),'ME',round(ME[k],3),'diff',round(diff[k],1),'d',np.round(d,3),'ex',round(d@ex,2),'bu',round(d@bu,2),'n',round(d@n,2))
