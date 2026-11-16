import Part
s=Part.read("output.step"); print(s.Volume,len(s.Faces),s.isValid(),len(s.Solids),s.BoundBox)
for n in "AB":
    f=Part.read("fixed_%s.step"%n); print(n,f.common(s).Volume)
import json,FreeCAD as App
V=App.Vector
B=json.load(open("task.json"))["poses"][0]["resolved"]["bolts"]
for b in B:
    p=V(*b["position"]);ax=V(*b["axis"])
    r=b["head_diameter"]/2+0.5
    st=6.0 if b["size"]=="M5" else 5.0
    c=Part.makeCylinder(r,5+ (6 if b["size"]=="M6" else 5)+30,p+ax*st,ax)
    print(b["id"],round(c.common(s).Volume,2))
