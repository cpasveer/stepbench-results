import Part, FreeCAD as App, json
V=App.Vector
s=Part.read("output.step")
T=json.load(open("task.json"))
for b in T["poses"][0]["resolved"]["bolts"]:
    if b["id"][0]!="A": continue
    ax=V(*b["axis"]);p=V(*b["position"])
    for g in (3.95,3.9,3.85,3.8):
        k=Part.makeCylinder(4.25,5,p+ax*g,ax)
        k2=Part.makeCylinder(2.5,g,p,ax)
        print(b["id"],g,round(s.common(k).Volume,3),round(s.common(k2).Volume,3))
