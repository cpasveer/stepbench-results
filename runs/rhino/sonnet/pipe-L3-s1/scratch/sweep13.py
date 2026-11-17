from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(T=10.0,PAD_HW=5.0,WALL_W=24.0,ROOT_W=24.0,W_TOP=12.0,TOP_W=-3.0,WALL_U_END=-30.0,TILT=2.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=True)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),r['share'],flush=True)
run('final cand tilt2')
run('tilt0 (no tilt, XB -23.5)',TILT=0.0)
