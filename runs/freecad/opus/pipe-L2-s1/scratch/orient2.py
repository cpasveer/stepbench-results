import numpy as np, json
exec(open('scratch/geo.py').read().split("print('B center")[0])
bu=RB[:,0];bv=RB[:,1]
rad=lambda p: np.sin(np.radians(p))*ey+np.cos(np.radians(p))*ez
a1=rad(-35);a3=rad(35)
crit=[-n,n,a1,a3]+[-rad(p) for p in range(-46,47,2)]
N=np.array(crit)
rng=np.random.default_rng(0)
D=rng.normal(size=(300000,3));D/=np.linalg.norm(D,axis=1)[:,None]
M=(D@N.T).max(axis=1)
for xs,ph in [(-8,10),(-4,10),(0,10),(-8,0),(0,0),(4,0)]:
    S=OA+xs*ex+41*rad(ph); PB=OB+6*n
    c=PB-S; c/=np.linalg.norm(c)
    # column must be within A deg of up (U=-d): -d.c >= cos
    up=-(D@c)
    ok=M<0.68
    for ang in (30,35,40):
        sel=ok&(up>=np.cos(np.radians(ang)))
        if sel.any():
            k=np.argmin(np.where(sel,M,9)); d=D[k]
            print(xs,ph,ang,'M',round(M[k],3),'d',np.round(d,3),'colang',round(np.degrees(np.arccos(up[k])),1)); break
    else: print(xs,ph,'none; best col angle with M<.68:',round(np.degrees(np.arccos(up[ok].max())),1))
