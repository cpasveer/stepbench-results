import Part, json, FreeCAD as App
V=App.Vector
s=Part.read("output.step")
print("bb",s.BoundBox, "faces",len(s.Faces))
for f in ["fixed_A.step","fixed_B.step"]:
    print(f, "overlap", s.common(Part.read(f)).Volume)
T=json.load(open("task.json"))
for b in T["poses"][0]["resolved"]["bolts"]:
    P=V(*b["position"]); ax=V(*b["axis"])
    t = 7 if b["size"]=="M5" else 8
    head=Part.makeCylinder(b["head_diameter"]/2+0.5, b["head_height"]+1, P+ax*t, ax)
    # crude key: short leg cylinder along axis up to 30mm
    key=Part.makeCylinder(2.9, 30, P+ax*(t+2), ax)
    print(b["id"], "head", round(s.common(head).Volume,3), "keyshort", round(s.common(key).Volume,3))
# overhang faces: normal z > 0.707 except top
for i,f in enumerate(s.Faces):
    u0,u1,v0,v1=f.ParameterRange
    n=f.normalAt((u0+u1)/2,(v0+v1)/2)
    if n.z>0.7: print("upface",i,f.Surface.__class__.__name__, round(f.Area,1), n, f.BoundBox.ZMax)
