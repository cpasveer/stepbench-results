import json, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); B=T["poses"][0]["resolved"]["bolts"]
s=Part.read("output.step")
print("solids",len(s.Solids),"valid",s.isValid(),"vol",s.Volume,"faces",len(s.Faces))
print("min edge",min(e.Length for e in s.Edges),"min face",min(f.Area for f in s.Faces))
for f in sorted(s.Faces,key=lambda f:f.Area)[:6]: print("  face",f.Surface.__class__.__name__,round(f.Area,3))
for e in sorted(s.Edges,key=lambda e:e.Length)[:6]: print("  edge",e.Curve.__class__.__name__,round(e.Length,3))
for n in "AB":
    fx=Part.read("fixed_%s.step"%n); print(n,"overlap",s.common(fx).Volume)
for b in B:
    p,ax=V(*b["position"]),V(*b["axis"])
    t=10.0 if b["id"][0]=="A" else 12.0
    head=Part.makeCylinder(b["head_diameter"]/2+0.5,b["head_height"]+1,p+ax*(t+0.01),ax)
    key=Part.makeCylinder(b["head_diameter"]/2+0.5,32,p+ax*(t+0.01),ax)
    print(b["id"],"head overlap",round(s.common(head).Volume,4),"dist",round(s.distToShape(Part.makeCylinder(b["head_diameter"]/2,b["head_height"],p+ax*(t+0.6),ax))[0],2),"axial col",round(s.common(key).Volume,3))
