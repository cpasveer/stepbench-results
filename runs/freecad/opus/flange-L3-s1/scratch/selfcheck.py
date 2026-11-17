import Part, FreeCAD as App, json, math
V=App.Vector
s=Part.read(globals().get("FN","output.step"))
T=json.load(open("task.json")); P=T["poses"][0]["resolved"]
print("solids",len(s.Solids),"faces",len(s.Faces),"vol",round(s.Volume),"valid",s.isValid())
for f in ["fixed_A.step","fixed_B.step"]:
    print(f,"overlap",round(s.common(Part.read(f)).Volume,4))
for b in P["bolts"]:
    p=V(*b["position"]); ax=V(*b["axis"]).normalize()
    hd=b["head_diameter"]; hh=b["head_height"]
    # find plate inner face distance: grip = thickness along axis
    line=Part.makeLine(p-ax*1, p+ax*60)
    # head seat at distance g
    g=None
    for t in [x*0.05 for x in range(0,1200)]:
        q=p+ax*t
        if not s.isInside(q+V(0,0,0)+ (ax.cross(V(0,0,1)).normalize()*( (hd/2+ (5.5/2))/2 )),1e-6,True):
            g=t;break
    head=Part.makeCylinder(hd/2+0.5, hh+1, p+ax*g, ax)
    tool=Part.makeCylinder(hd/2, 30, p+ax*(g+hh), ax)
    print(b["id"],"grip",round(g,2),"head clash",round(s.common(head).Volume,3),"tool clash",round(s.common(tool).Volume,3))
# overhang faces (print up = -z)
d=V(0,0,-1)
bad=0
for f in s.Faces:
    if f.Surface.__class__.__name__=="Cylinder": continue
    u0,u1,v0,v1=f.ParameterRange
    n=f.normalAt((u0+u1)/2,(v0+v1)/2)
    c=-n.dot(d)  # >0 means facing down in print
    if c>math.cos(math.radians(45))+1e-6:
        zmax=f.BoundBox.ZMax; zmin=f.BoundBox.ZMin
        onbed = abs(zmin-36.5)<1e-3 and abs(zmax-36.5)<1e-3
        if not onbed:
            bad+=1; print("OVERHANG face area",round(f.Area,2),"n",n, f.BoundBox)
print("overhang faces",bad)
small=[f for f in s.Faces if f.Area<1.0]
print("small faces",[round(f.Area,3) for f in small])
print("short edges",[round(e.Length,3) for e in s.Edges if e.Length<1.0])
