import sys; sys.path.insert(0,'.')
import submission as S
from build123d import *
p=import_step('output.step')
A=import_step('fixed_A.step'); B=import_step('fixed_B.step')
print('vol',p.volume,'faces',len(p.faces()))
for n,f in (('A',A),('B',B)):
    i=p.intersect(f); print('overlap',n, i.volume if i else 0)
up=-S.V  # print up direction
bed=None
tot=0
for f in p.faces():
    a=f.area
    # sample
    bad=0; N=0
    for u in (0.1,0.3,0.5,0.7,0.9):
        for v in (0.1,0.3,0.5,0.7,0.9):
            try:
                pt=f.position_at(u,v); nrm=f.normal_at(u,v)
            except Exception: continue
            N+=1
            c=nrm.dot(up)
            if c< -0.7072: 
                # on bed?
                if abs((pt-S.A0).dot(S.V)-S.BED)>1e-3: bad+=1
    if bad: print('overhang face',f.geom_type,round(a,1),bad,N, f.center())
