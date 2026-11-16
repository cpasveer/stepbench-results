import math
from build123d import *
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN
p=import_step('output.step'); sol=p.solids()[0]
def inside(pt):
    c=BRepClass3d_SolidClassifier(sol.wrapped, pt.to_pnt(), 1e-6); return c.State()==TopAbs_IN
faces=p.faces()
for e in p.edges():
    fs=[f for f in faces if any(e.is_same(x) for x in f.edges())]
    if len(fs)!=2: continue
    for par in (0.2,0.5,0.8):
        P=e.position_at(par); T=e.tangent_at(par)
        ts=[]
        for f in fs:
            n=f.normal_at(P)
            t=T.cross(n).normalized()
            # choose direction pointing into face: test point on face
            q=P+t*0.05
            if (f.find_intersection_points if False else None): pass
            d1=(f.center()-P)
            # project candidate and check distance to face
            if f.distance_to(P+t*0.05) > f.distance_to(P-t*0.05): t=-t
            ts.append(t)
        a=math.degrees(math.acos(max(-1,min(1,ts[0].dot(ts[1])))))
        b=(ts[0]+ts[1])
        if b.length<1e-6: continue
        if not inside(P+b.normalized()*0.05): a=360-a
        if a<88: print(round(a,1), [round(x,2) for x in (P.X,P.Y,P.Z)], e.geom_type)
