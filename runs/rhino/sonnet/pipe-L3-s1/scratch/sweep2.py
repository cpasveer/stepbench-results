from fe2 import *
g=Grid(1.0)
base=default_params()
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),flush=True)
    return r
for W,R,T in [(16,28,-6),(20,20,-6),(20,28,-6),(20,28,-8),(24,24,-8),(16,32,-4),(18,24,-6)]:
    run('W%d R%d top%d'%(W,R,T),WALL_W=float(W),ROOT_W=float(R),TOP_W=float(T))
for Tp in (5.0,6.0):
    run('W20 R20 T%d'%Tp,WALL_W=20.0,ROOT_W=20.0,T=Tp)
