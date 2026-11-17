import FreeCAD as App, Part, math, sys
V=App.Vector
s=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/output.step")
faces=s.Faces
def overhang(u, thr=45.0):
    u=u*(1.0/u.Length)
    # bed = lowest point along u
    hmin=min(v.Point.dot(u) for v in s.Vertexes)
    tot=0.0; worst=0.0
    for f in faces:
        # sample normal at centre
        try:
            c=f.CenterOfMass
            n=f.normalAt(*f.Surface.parameter(c)) if False else None
        except Exception:
            pass
        # use uv midpoint
        u0,u1,v0,v1=f.ParameterRange
        p=f.valueAt((u0+u1)/2,(v0+v1)/2)
        n=f.normalAt((u0+u1)/2,(v0+v1)/2)
        if f.Orientation=="Reversed": n=n*-1
        d=n.dot(u)   # >0 means facing up
        if d < -math.cos(math.radians(thr)):   # downward within thr of straight down
            h=f.CenterOfMass.dot(u)
            if h-hmin>0.3:   # not on the bed
                tot+=f.Area
    return tot
if True:
    cands={}
    import itertools
    for name,u in [("a(nA)",V(0.7384553,0.6743024,0)),("-a",V(-0.7384553,-0.6743024,0)),
                   ("nB",V(-0.4786919,-0.8779830,0)),("-nB",V(0.4786919,0.8779830,0)),
                   ("Z",V(0,0,1)),("-Z",V(0,0,-1)),("tA",V(-0.6743024,0.7384553,0)),("-tA",V(0.6743024,-0.7384553,0))]:
        print(name, round(overhang(u),1))
    best=None
    # coarse sphere sweep
    N=0
    for i in range(0,181,6):
        for j in range(0,360,6):
            th=math.radians(i); ph=math.radians(j)
            u=V(math.sin(th)*math.cos(ph), math.sin(th)*math.sin(ph), math.cos(th))
            oh=overhang(u)
            if best is None or oh<best[0]: best=(oh,i,j)
    print("best sphere sample:", best)
