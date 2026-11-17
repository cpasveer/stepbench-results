import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import *
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/model4.py").read().split("plate_pt =")[0].split("E=2500.0; G=E/2.6")[1].replace("r1=A","#"), globals()) if False else None
E=2500.0; G=E/2.6
r1=A["A1"]; r3=A["A3"]
def S(r,s,rad=38): return P0 + s*AX + rad*r
def xyz(u,v,n): return CB+UB*u+ZB*v+NB*n
def minstiff(pts, mem, fixed, load=0):
    n=len(pts); K=np.zeros((6*n,6*n))
    for (i,j,Aa,I,J) in mem:
        ke=beam_k(pts[i],pts[j],E,G,Aa,I,I,J)
        dof=list(range(6*i,6*i+6))+list(range(6*j,6*j+6)); K[np.ix_(dof,dof)]+=ke
    free=[d for d in range(6*n) if d//6 not in fixed]
    Kff=K[np.ix_(free,free)]; Cm=np.zeros((3,3))
    for a_ in range(3):
        for b_ in range(3):
            F=np.zeros(len(free)); F[free.index(6*load+a_)]=1
            Cm[a_,b_]=np.linalg.solve(Kff,F)[free.index(6*load+b_)]
    w=np.linalg.eigvalsh(np.linalg.inv(Cm)); return round(float(w.min()),1), round(float(w.max()),1)
C = xyz(0,0,4)
R1 = xyz(1,8,4); R3 = xyz(0,-14.5,4)
legs = [("S1-6",S(r1,-6)),("S1+6",S(r1,6)),("S3c",S(r3,0))]
pts=[C,R1,R3]+[l[1] for l in legs]
big=1e3
mem=[(0,1,400,4*400.0/4 *0+ 2000.0, 2000.0),(0,2,400,2000.0,2000.0)]  # rigid-ish links plate centre to roots
mem=[(0,1,1600.0,1600*400/12.0,1600*400/12.0*0+2*1600*400/12.0),(0,2,1600.0,1600*400/12.0,2*1600*400/12.0)]
mem=[(0,1,1600.0,1600*400/12.0,2*1600*400/12.0),(0,2,1600.0,1600*400/12.0,2*1600*400/12.0)]
mem += [(1,3,100,833,141),(1,4,100,833,141),(2,5,100,833,141)]
print("3 legs from separate roots, min/max eig", minstiff(pts,mem,{3,4,5}))
