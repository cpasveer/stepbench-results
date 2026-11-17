from fe2 import *
g=Grid(1.0)
base=default_params()
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,{k:(round(float(v),3) if not isinstance(v,(dict,np.ndarray)) else v) for k,v in r.items()},'k/printed',round(float(r['k']/r['printed']*1000),2),flush=True)
run('base')
run('W16',WALL_W=16.0,ROOT_W=16.0)
run('W12 top-2',TOP_W=-2.0)
run('W12 R24',ROOT_W=24.0)
run('W16 top-2',WALL_W=16.0,ROOT_W=16.0,TOP_W=-2.0)
run('W12 R24 top0',ROOT_W=24.0,TOP_W=0.0)
run('W20 top-6',WALL_W=20.0,ROOT_W=20.0)
