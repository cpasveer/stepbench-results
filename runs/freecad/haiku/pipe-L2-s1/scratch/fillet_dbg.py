import math
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
def fn(f):
    if f.Surface.__class__.__name__ != "Plane": return None
    u0,u1,v0,v1=f.ParameterRange
    return f.normalAt((u0+u1)/2,(v0+v1)/2)
for e in s.Edges:
    fs = s.ancestorsOfType(e, Part.Face)
    if len(fs)!=2: continue
    n1=fn(fs[0]); n2=fn(fs[1])
    if n1 is None or n2 is None: continue
    ang=math.degrees(math.acos(max(-1,min(1,n1.dot(n2)))))
    if ang>2 and 180-ang<50 and e.Length>1.0:
        p0=e.Vertexes[0].Point; p1=e.Vertexes[-1].Point
        print("EDGE", [round(v,2) for v in p0], [round(v,2) for v in p1], round(e.Length,2), round(ang,2))
