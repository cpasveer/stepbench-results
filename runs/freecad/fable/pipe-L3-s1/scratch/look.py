import json, math, os
import FreeCAD as App, Part
V=App.Vector
HERE=os.getcwd()
T=json.load(open("task.json")); pose=T["poses"][0]
s=Part.read("output.step")
print("valid",s.isValid(),"solids",len(s.Solids),"vol",s.Volume,"area",s.Area)
ra=pose["placements"]["A"]["rotation"]; D=V(ra[0][0],ra[1][0],ra[2][0]); up=D*-1
for name in ("fixed_A.step","fixed_B.step"):
    f=Part.read(name); c=s.common(f); print(name,"common vol",c.Volume, "dist",s.distToShape(f)[0])
tot=0
for i,f in enumerate(s.Faces):
    u0,u1,v0,v1=f.ParameterRange
    n=f.normalAt((u0+u1)/2,(v0+v1)/2)
    dn=n.dot(up*-1)
    typ=f.Surface.__class__.__name__
    zs=[v.Point.dot(up) for v in f.Vertexes]
    print(i,typ,"area %.1f"%f.Area,"down %.3f"%dn,"h %.1f..%.1f"%(min(zs),max(zs)))
# tool columns
for b in pose["resolved"]["bolts"]:
    p=V(*b["position"]); ax=V(*b["axis"])
    # find grip: 
    hd=b["head_diameter"]; hh=b["head_height"]
    for g in (4.0,4.5):
        cyl=Part.makeCylinder(hd/2,30+hh+1,p+ax*(g+0.01),ax)
        print(b["id"],g,"col common",round(s.common(cyl).Volume,3))
