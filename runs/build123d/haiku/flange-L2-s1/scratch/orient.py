import pathlib, math
from build123d import *
p = pathlib.Path(__file__).parent.parent / "output.step"
s = import_step(str(p)).solids()[0]
faces=[]
for f in s.faces():
    c=f.center(); n=f.normal_at(c); faces.append((f.area, (n.X,n.Y,n.Z), (c.X,c.Y,c.Z)))
def score(u, bed_tol=0.2):
    # height of each face centroid along u; bed = lowest faces
    hs=[ (c[0]*u[0]+c[1]*u[1]+c[2]*u[2]) for a,n,c in faces]
    hmin=min(hs)
    tot=0.0; worst=0.0
    for (a,n,c),h in zip(faces,hs):
        nu = n[0]*u[0]+n[1]*u[1]+n[2]*u[2]
        if nu < -math.sin(math.radians(45)) and h-hmin > bed_tol:
            tot += a
            worst = max(worst, math.degrees(math.asin(min(1,-nu))))
    return tot, worst
best=[]
for ti in range(0,181,5):
    for pi in range(0,360,5):
        t=math.radians(ti); p_=math.radians(pi)
        u=(math.sin(t)*math.cos(p_), math.sin(t)*math.sin(p_), math.cos(t))
        tot,w=score(u)
        best.append((tot,w,ti,pi))
best.sort()
for r in best[:8]: print("area_over45=%.1f worst=%.1f  theta=%d phi=%d"%r)
print("z-up:", [round(x,1) for x in score((0,0,1))])
