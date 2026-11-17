from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(T=10.0,PAD_HW=5.0,WALL_W=24.0,ROOT_W=24.0,W_TOP=12.0,TOP_W=-3.0,WALL_U_END=-30.0,TILT=2.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),flush=True)
run('T11',T=11.0)
run('T12',T=12.0)
run('T11 base28',T=11.0,WALL_W=28.0,ROOT_W=28.0)
run('T11 tilt1.2',T=11.0,TILT=1.2)
run('T11 NMIN.3',T=11.0,NMIN=0.3)
