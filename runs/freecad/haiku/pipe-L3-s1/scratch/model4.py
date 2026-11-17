import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import *
E=2500.0; G=E/2.6
r1=A["A1"]; r3=A["A3"]
def S(r, s, rad=36): return P0 + s*AX + rad*r
def minstiff(pts, mem, fixed, load=0):
    C=np.zeros((3,3))
    for d in range(3):
        pass
    res=[]
    # full 3x3 compliance from unit loads
    n=len(pts); K=np.zeros((6*n,6*n))
    for (i,j,Aa,I,J) in mem:
        ke=beam_k(pts[i],pts[j],E,G,Aa,I,I,J)
        dof=list(range(6*i,6*i+6))+list(range(6*j,6*j+6)); K[np.ix_(dof,dof)]+=ke
    free=[d for d in range(6*n) if d//6 not in fixed]
    Kff=K[np.ix_(free,free)]
    Cm=np.zeros((3,3))
    for a_ in range(3):
        for b_ in range(3):
            F=np.zeros(len(free)); F[free.index(6*load+a_)]=1
            u=np.linalg.solve(Kff,F)
            Cm[a_,b_]=u[free.index(6*load+b_)]
    Kst=np.linalg.inv(Cm)
    return round(float(np.linalg.eigvalsh(Kst).min()),1), round(float(np.linalg.eigvalsh(Kst).max()),1)
plate_pt = CB + UB*(-4.0) + ZB*(-4.0) + NB*4.0
# calibration: current post design (10x10 post, 2 beams)
top = CB - 4*UB - 4*ZB + 48*NB
pts=[plate_pt, top, S(r1,36), S(r3,36)]
print("post design min/max eig", minstiff(pts,[(0,1,100,833,141),(1,2,100,833,141),(1,3,100,833,141)],{2,3}))
for hh in [28.0, 32.0]:
    for a in [8.0,10.0]:
        hub = CB + UB*(-4.0) + ZB*(-4.0) + NB*hh
        legs=[S(r1,0), S(r3,-6), S(r3,6)]
        pts=[plate_pt, hub]+legs
        mem=[(0,1,a*a,a**4/12,0.141*a**4)]+[(1,2+i,a*a,a**4/12,0.141*a**4) for i in range(3)]
        print("hub",hh,"leg",a, "min/max eig", minstiff(pts,mem,{2,3,4}))
