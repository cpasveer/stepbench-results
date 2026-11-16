import Part, json, math
import FreeCAD as App
s=Part.read("output.step")
R=json.load(open("task.json"))["poses"][0]["placements"]["A"]["rotation"]
up=App.Vector(-R[0][1],-R[1][1],0)   # build dir = -local y
for i,f in enumerate(s.Faces):
    worst=0;
    u0,u1,v0,v1=f.ParameterRange
    for a in range(21):
        for b in range(21):
            u=u0+(u1-u0)*a/20; v=v0+(v1-v0)*b/20
            n=f.normalAt(u,v); d=-n.dot(up)
            if d>worst: worst=d
    if worst>0.01:
        print(i,f.Surface.__class__.__name__,round(f.Area,1),"overhang from vertical",round(math.degrees(math.asin(min(1,worst))),1), f.BoundBox)
