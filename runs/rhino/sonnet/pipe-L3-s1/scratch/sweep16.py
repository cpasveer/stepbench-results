from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(T=11.0,PAD_HW=5.0,WALL_W=24.0,ROOT_W=24.0,W_TOP=12.0,TOP_W=-3.0,WALL_U_END=-30.0,TILT=2.0,DRAFT=math.tan(math.radians(5)))
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),flush=True)
run('as built (end-30 T11)')
run('end-34',WALL_U_END=-34.0)
run('end-34 T10',WALL_U_END=-34.0,T=10.0)
run('end-34 T10 W22',WALL_U_END=-34.0,T=10.0,WALL_W=22.0,ROOT_W=22.0)
run('end-34 T9',WALL_U_END=-34.0,T=9.0)
