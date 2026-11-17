from fe2 import *
g=Grid(1.0)
base=default_params()
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),flush=True)
    return r
for W,R,T,hw in [(16,20,9,6.5),(18,18,8,6.5),(16,16,10,6.5),(14,14,10,6.5),(16,16,9,5.0),(16,16,9,8.0)]:
    run('W%d R%d T%d hw%.1f'%(W,R,T,hw),WALL_W=float(W),ROOT_W=float(R),T=float(T),PAD_HW=hw)
