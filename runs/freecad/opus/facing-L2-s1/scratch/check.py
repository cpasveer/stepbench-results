import Part
s=Part.read("output.step"); print("solids",len(s.Solids),"valid",s.isValid(),"vol",s.Volume, s.BoundBox)
for f in ["fixed_A.step","fixed_B.step"]:
    print(f, s.common(Part.read(f)).Volume)
import json, FreeCAD as App
T=json.load(open("task.json"))
for b in T["poses"][0]["resolved"]["bolts"]:
    p=App.Vector(*b["position"]); a=App.Vector(*b["axis"])
    q=p+a*3; off=App.Vector(3.5 if b["size"]=="M4" else 4.0,0,0)
    print(b["id"], s.isInside(q,1e-6,True), s.isInside(q+off,1e-6,True), s.isInside(q-off,1e-6,True))
for b in T["poses"][0]["resolved"]["bolts"]:
    p=App.Vector(*b["position"]); a=App.Vector(*b["axis"]); g=6.0
    head=Part.makeCylinder(b["head_diameter"]/2+0.5, b["head_height"]+1, p+a*g, a)
    print(b["id"],"headclash", round(s.common(head).Volume,4))
