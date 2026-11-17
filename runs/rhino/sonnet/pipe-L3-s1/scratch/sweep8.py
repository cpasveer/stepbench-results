from fe2 import *
g=Grid(1.0)
base=default_params()
base.update(XB=-20.5,T=9.0,PAD_HW=5.0)
def run(tag,**kw):
    p=dict(base);p.update(kw)
    occ=build(g,p);r=solve(occ,g,regions=True)
    print(tag,'k',round(float(r['k']),2),'printed',round(float(r['printed'])),'k/p',round(float(r['k']/r['printed']*1000),2),r['share'],flush=True)
run('W14 top-6 blk xbt',WALL_W=14.0,ROOT_W=14.0,TOP_W=-6.0)
run('W14 top-6 blk-6',WALL_W=14.0,ROOT_W=14.0,TOP_W=-6.0,BLOCK_TOP=-6.0,BLOCK_UMAX=-17.4)
run('W14 top-6 blk-3',WALL_W=14.0,ROOT_W=14.0,TOP_W=-6.0,BLOCK_TOP=-3.0,BLOCK_UMAX=-17.4)
run('W14 top-6 blk0',WALL_W=14.0,ROOT_W=14.0,TOP_W=-6.0,BLOCK_TOP=0.0,BLOCK_UMAX=-17.4)
run('W12 top-6 blk-6',WALL_W=12.0,ROOT_W=12.0,TOP_W=-6.0,BLOCK_TOP=-6.0,BLOCK_UMAX=-17.4)
