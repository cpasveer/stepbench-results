from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(XB=-20.5,WALL_W=14.0,ROOT_W=14.0,T=9.0,PAD_HW=5.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=True)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),r['share'],r['mode'],'nwash',r['nwash'],flush=True)
run('flat-6',TOP_W=-6.0)
run('flat-2',TOP_W=-2.0)
run('flat0',TOP_W=0.0)
run('flat0 slope0',TOP_W=0.0,SLOPE=0.0)
run('slope .3 top0',TOP_W=0.0,SLOPE=0.3)
