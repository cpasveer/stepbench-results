# own pre-check: bolts, heads, tool cylinders, fixed parts, seat extents
import Part, json, math, FreeCAD as App
V=App.Vector
T=json.load(open("task.json"))["poses"][0]["resolved"]
s=Part.read("output.step")
print("solids",len(s.Solids),"valid",s.isValid(),"vol",round(s.Volume,1),"faces",len(s.Faces),"bbox",s.BoundBox)
for nm in "AB":
    f=Part.read("fixed_%s.step"%nm); print("overlap",nm,round(s.common(f).Volume,4))
import os
grip={"M5":float(os.environ.get("TA","8")),"M6":float(os.environ.get("TB","8"))}
for b in T["bolts"]:
    p=V(*b["position"]); ax=V(*b["axis"]); hd=b["head_diameter"]; hh=b["head_height"]; g=grip[b["size"]]
    head=Part.makeCylinder(hd/2+0.5,hh+1.0,p+ax*(g+0.01),ax)
    tool=Part.makeCylinder(hd/2,30,p+ax*(g+hh),ax)
    ring=Part.makeCylinder(hd/2,0.02,p+ax*(g-0.01),ax)
    print(b["id"],"head",round(s.common(head).Volume,4),"tool",round(s.common(tool).Volume,4),
          "dist head",round(s.distToShape(head)[0],3),"tool",round(s.distToShape(tool)[0],3))
small=[(round(f.Area,3)) for f in s.Faces if f.Area<1.0]
print("small faces",small)
print("short edges",[round(e.Length,3) for e in s.Edges if e.Length<0.5])
