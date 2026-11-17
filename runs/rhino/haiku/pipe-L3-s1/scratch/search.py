import math
from frame import nrm
C=(19.549595153015062,49.68566626427389,52.2472790015625); N=(0.3885173161164798,0.9214414224885092,0.0)
U=(0.0,0.0,1.0); V=(0.9214414224885092,-0.3885173161164798,0.0)
A1=(0.7093118905965579,-0.6509870200868549,0.27035632165831364); A3=(-0.3771547728198181,-0.6713858946506742,0.6379617989560074)
m=[A1[i]+A3[i] for i in range(3)]; l=nrm(m); m=[x/l for x in m]
P0=(21.0,136.5,-7.0)
E=[P0[i]+36*m[i] for i in range(3)]
heads=[(-12.5,-12.5),(12.5,-12.5),(12.5,12.5),(-12.5,12.5)]
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def unit(a): l=nrm(a); return tuple(x/l for x in a)
def uvs(p):
    w=[p[i]-C[i] for i in range(3)]
    return dot(w,U), dot(w,V), dot(w,N)
def clearance(u0,v0,a,phi, top=None):
    R=[C[i]+u0*U[i]+v0*V[i] for i in range(3)]
    d=unit([E[i]-R[i] for i in range(3)])
    L=nrm([E[i]-R[i] for i in range(3)])
    e1=unit(cross(d,(0,0,1))) if abs(d[2])<0.99 else (1,0,0)
    e2=unit(cross(d,e1))
    # rotate cross-section by phi
    c,s=math.cos(phi),math.sin(phi)
    f1=tuple(c*e1[i]+s*e2[i] for i in range(3))
    f2=tuple(-s*e1[i]+c*e2[i] for i in range(3))
    h=a/2.0
    worst=1e9; worst_zone=1e9
    # sample along arm axis: keep s in [5,39] region of interest
    steps=int(L/0.5)
    for k in range(steps+1):
        t=k*0.5
        P=[R[i]+d[i]*t for i in range(3)]
        for x in [-h+j*(a/10.0) for j in range(11)]:
            for y in [-h+j*(a/10.0) for j in range(11)]:
                q=[P[i]+x*f1[i]+y*f2[i] for i in range(3)]
                uu,vv,ss=uvs(q)
                if ss<5.0 or ss>39.0: continue
                for (hu,hv) in heads:
                    dd=math.hypot(uu-hu,vv-hv)
                    if dd<worst: worst=dd
                    if ss<10.0 and dd<worst_zone: worst_zone=dd
    return worst, worst_zone, L, d
if __name__=="__main__":
    best=[]
    for a in [14.0,16.0,18.0]:
        for phi in [0.0, math.pi/4]:
            for u0 in range(-8,9,2):
                for v0 in range(-8,9,2):
                    w,wz,L,d=clearance(float(u0),float(v0),a,phi)
                    best.append((min(w-3.5, wz-4.0), a, round(phi*180/math.pi), u0, v0, round(w,2), round(wz,2), round(L,1)))
    best.sort(reverse=True)
    for b in best[:12]: print(b)
