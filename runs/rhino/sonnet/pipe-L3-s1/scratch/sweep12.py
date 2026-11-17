from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(XB=-20.5,T=9.0,PAD_HW=5.0,WALL_W=24.0,ROOT_W=24.0,W_TOP=12.0,TOP_W=-3.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=True)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),r['share'],flush=True)
run('end-30 T9',WALL_U_END=-30.0)
run('end-30 T10',WALL_U_END=-30.0,T=10.0)
run('end-30 T10 top-4',WALL_U_END=-30.0,T=10.0,TOP_W=-4.0)
run('end-32 T11',WALL_U_END=-32.0,T=11.0)
