import json, Part, FreeCAD as App
V=App.Vector
P=json.load(open("task.json"))["poses"][0]
s=Part.read("output.step")
for b in P["resolved"]["bolts"]:
    p=V(*b["position"]); a=V(*b["axis"]); hd=b["head_diameter"]; hh=b["head_height"]
    # find grip: distance along axis where the part ends
    line=Part.makeLine(p+a*0.01+V(hd/2-0.2,0,0)*0, p+a*30)
    grip=3.2 if b["id"][0]=="A" else 6.203
    head=Part.makeCylinder(hd/2+0.5,hh+1,p+a*(grip+1e-4),a)
    tool=Part.makeCylinder(hd/2,30,p+a*(grip+hh),a)
    print(b["id"],"head-zone overlap %.5f mm3, dist %.4f | tool overlap %.5f, dist %.4f"%(s.common(head).Volume,s.distToShape(head)[0],s.common(tool).Volume,s.distToShape(tool)[0]))
