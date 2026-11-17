import math
import search as S
from frame import nrm
def unit(a): l=nrm(a); return tuple(x/l for x in a)
def cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
D=unit((0.2487956429525063,0.5900653629615055,0.7680648380709507))
m=S.m; t=unit(cross(D,m)); P0=S.P0
res=[]
for ax, arc in [(0,0),(-6,0),(0,8),(-6,8),(6,0),(6,-8),(0,-8)]:
    E=[P0[i]+36*m[i]+ax*D[i]+arc*t[i] for i in range(3)]
    S.E=E
    for a in [12.0,14.0,16.0]:
        for phi in [0.0, math.pi/4]:
            for u0 in range(-10,11,2):
                for v0 in range(-10,11,2):
                    w,wz,L,d=S.clearance(float(u0),float(v0),a,phi)
                    res.append((round(min(w-3.5, wz-4.0),2), a, round(phi*180/math.pi), ax, arc, u0, v0, round(w,2), round(wz,2), round(L,1)))
res.sort(reverse=True)
for r in res[:12]: print(r)
