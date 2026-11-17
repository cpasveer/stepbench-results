import math
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
def nrm(f):
    u0,u1,v0,v1=f.ParameterRange
    return f.normalAt((u0+u1)/2,(v0+v1)/2)
for e in s.Edges:
    fs = s.ancestorsOfType(e, Part.Face)
    if len(fs)!=2: continue
    f1,f2=fs
    if f1.Surface.__class__.__name__!="Plane" or f2.Surface.__class__.__name__!="Plane": continue
    n1=nrm(f1); n2=nrm(f2)
    ang = math.degrees(math.acos(max(-1,min(1,n1.dot(n2)))))
    if 180-ang < 45 and ang>2:
        p0=e.Vertexes[0].Point; p1=e.Vertexes[-1].Point
        print("SHALLOW p0", [round(v,2) for v in p0], "p1", [round(v,2) for v in p1], "len", round(e.Length,2), "n1", [round(v,3) for v in n1], "n2", [round(v,3) for v in n2], "areas", round(f1.Area,1), round(f2.Area,1))
