import json, math, Part, FreeCAD as App
V=App.Vector
P=json.load(open("task.json"))["poses"][0]
s=Part.read("output.step")
for b in P["resolved"]["bolts"]:
    p=V(*b["position"]); a=V(*b["axis"]); d=4.4 if b["id"][0]=="A" else 5.5
    t=3.2 if b["id"][0]=="A" else 6.19
    # smallest wall: grow a cylinder until it pokes out of the part within the plate thickness
    for w in (0.8,0.85,0.9,1.0,1.2,1.5,2.0):
        c=Part.makeCylinder(d/2+w,t-0.2,p+a*0.1,a)
        out=c.cut(s).Volume- math.pi*(d/2)**2*(t-0.2)
        if out>1e-3: print(b["id"],"wall <",w); break
    else: print(b["id"],"wall >= 2.0")
