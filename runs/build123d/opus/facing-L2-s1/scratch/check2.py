from build123d import *
p=import_step('output.step')
A=import_step('fixed_A.step'); B=import_step('fixed_B.step')
print('vol',p.volume,'faces',len(p.faces()))
for n,f in (('A',A),('B',B)):
    i=p.intersect(f); print('overlap',n, i.volume if i else 0)
up=Vector(0,0,1)
for f in p.faces():
    bad=0;N=0
    for u in (0.05,0.3,0.5,0.7,0.95):
        for v in (0.05,0.3,0.5,0.7,0.95):
            try: pt=f.position_at(u,v); nrm=f.normal_at(u,v)
            except Exception: continue
            N+=1
            if nrm.dot(up)< -0.7072 and pt.Z>-93.99: bad+=1
    if bad: print('overhang',f.geom_type,round(f.area,1),bad,N,f.center())
