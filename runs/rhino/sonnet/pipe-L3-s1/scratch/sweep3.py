from fe2 import *
g=Grid(1.0)
base=default_params()
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=True)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),r.get('share'),r.get('mode'),flush=True)
    return r
run('W20 T8',WALL_W=20.0,ROOT_W=20.0,T=8.0)
run('W16 T9',WALL_W=16.0,ROOT_W=16.0,T=9.0)
run('W12 T9',WALL_W=12.0,ROOT_W=12.0,T=9.0)
run('W16 T8 win6',WALL_W=16.0,ROOT_W=16.0,T=8.0,WIN=6.0)
