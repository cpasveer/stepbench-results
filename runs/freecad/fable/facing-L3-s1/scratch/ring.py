import json, math, Part, FreeCAD as App
V=App.Vector
P=json.load(open("task.json"))["poses"][0]
s=Part.read("output.step")
for b in P["resolved"]["bolts"]:
    p=V(*b["position"]); a=V(*b["axis"]); hd=b["head_diameter"]
    d=4.4 if b["id"][0]=="A" else 5.5
    ring=Part.makeCylinder(hd/2,0.02,p+a*0.004,a).cut(Part.makeCylinder(d/2,1,p-a*0.5,a))
    print(b["id"],"ring coverage %.3f"%(ring.common(s).Volume/ring.Volume), "head-clear common %.5f"%s.common(Part.makeCylinder(hd/2+0.5,b["head_height"]+1,p+a*(3.2 if b["id"][0]=="A" else 6.1971),a)).Volume)
