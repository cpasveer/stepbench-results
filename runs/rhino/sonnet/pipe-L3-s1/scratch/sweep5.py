from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(XB=-20.5)   # tilt effect: bed ~ -19.5 at plate end (-21 at pads): approximate with -20.5
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=False)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),flush=True)
    return r
run('W16 T9 flat-6',WALL_W=16.0,ROOT_W=16.0,T=9.0,PAD_HW=5.0)
run('W14 T9 slope .3 top0',WALL_W=14.0,ROOT_W=14.0,T=9.0,PAD_HW=5.0,TOP_W=0.0,SLOPE=0.3)
run('W14 T9 slope .4 top4',WALL_W=14.0,ROOT_W=14.0,T=9.0,PAD_HW=5.0,TOP_W=4.0,SLOPE=0.4)
run('W16 T9 slope .3 top0',WALL_W=16.0,ROOT_W=16.0,T=9.0,PAD_HW=5.0,TOP_W=0.0,SLOPE=0.3)
run('W12 R20 T9 slope .4 top4',WALL_W=12.0,ROOT_W=20.0,T=9.0,PAD_HW=5.0,TOP_W=4.0,SLOPE=0.4)
