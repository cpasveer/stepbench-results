from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(XB=-20.5,T=9.0,PAD_HW=5.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=True)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),r['share'],flush=True)
run('e-40 W14 top0',WALL_U_END=-40.0,WALL_W=14.0,ROOT_W=14.0,TOP_W=0.0)
run('e-46 W16 top0',WALL_U_END=-46.0,WALL_W=16.0,ROOT_W=16.0,TOP_W=0.0)
run('e-40 W12 top3',WALL_U_END=-40.0,WALL_W=12.0,ROOT_W=12.0,TOP_W=3.0)
run('e-46 W12 top6',WALL_U_END=-46.0,WALL_W=12.0,ROOT_W=12.0,TOP_W=6.0)
run('e-34 W12 top-3',WALL_U_END=-34.0,WALL_W=12.0,ROOT_W=12.0,TOP_W=-3.0)
