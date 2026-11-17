import json, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); P=T["poses"][0]
s=Part.read("output.step")
c=V(*P["placements"]["B"]["origin"])
for r in (41.25,41.0,40.5,40,39):
    disc=Part.makeCylinder(r,0.02,c-V(0,0,0.02),V(0,0,1))
    slab=Part.makeBox(300,300,0.02,V(-300,0,-25.52))
    top=s.common(slab)
    print(r,"seat area",top.Volume/0.02,"outside",top.cut(disc).Volume/0.02)
out=top.cut(Part.makeCylinder(41.25,0.02,c-V(0,0,0.02),V(0,0,1)))
for so in out.Solids: print(so.Volume/0.02, so.CenterOfMass, so.BoundBox)
