from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(PAD_HW=5.0,TILT=2.0,WALL_U_END=-28.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),flush=True)
for T in (9.0,11.0,13.0):
    run('rect W12 top-6 T%d'%T,WALL_W=12.0,ROOT_W=12.0,TOP_W=-6.0,T=T)
for T in (11.0,13.0):
    run('rect W14 top-6 T%d'%T,WALL_W=14.0,ROOT_W=14.0,TOP_W=-6.0,T=T,WALL_U_END=-32.0)
