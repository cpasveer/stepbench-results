import json, math, os
import FreeCAD as App, Part
V=App.Vector
H=os.getcwd()
T=json.load(open("task.json")); P=T["poses"][0]
s=Part.read("output.step"); A=Part.read("fixed_A.step"); B=Part.read("fixed_B.step")
print("solids",len(s.Solids),"valid",s.isValid(),"vol",s.Volume,"faces",len(s.Faces),"edges",len(s.Edges))
print("bbox B",B.BoundBox); print("bbox part",s.BoundBox)
print("common A",s.common(A).Volume,"common B",s.common(B).Volume)
RA=P["placements"]["A"]["rotation"]; d=V(RA[0][0],RA[1][0],RA[2][0])
for i,f in enumerate(s.Faces):
    u0,u1,v0,v1=f.ParameterRange
    worst=1
    for a in range(7):
        for b in range(7):
            u=u0+(u1-u0)*(a+.5)/7; v=v0+(v1-v0)*(b+.5)/7
            if f.isPartOfDomain(u,v):
                worst=min(worst,f.normalAt(u,v).dot(d))
    ang=math.degrees(math.asin(max(-1,min(1,-worst))))
    print(i,f.Surface.__class__.__name__,"area %.2f"%f.Area,"down-angle %.1f"%ang, "zmin(d) %.2f"%min(v.Point.dot(d) for v in f.Vertexes) if f.Vertexes else "")
print("min edge",min(e.Length for e in s.Edges))
print(sorted(round(e.Length,3) for e in s.Edges)[:8])
