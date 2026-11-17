import json, FreeCAD as App, Part
V=App.Vector
T=json.load(open("task.json")); pose=T["poses"][0]
s=Part.read("output.step")
for b in pose["resolved"]["bolts"]:
    p=V(*b["position"]); ax=V(*b["axis"]); hd=b["head_diameter"]; hh=b["head_height"]
    g=5.0 if b["id"][0]=="B" else 4.0
    head=Part.makeCylinder(hd/2+0.5,hh+1,p+ax*(g+0.001),ax)
    col=Part.makeCylinder(hd/2,hh+30,p+ax*(g+0.001),ax)
    print(b["id"],"head-clear common %.4f dist %.3f | tool col common %.4f dist %.3f"%(s.common(head).Volume, 0, s.common(col).Volume, 0))
    # distance of beam etc to column ignoring the seat: shrink start by 0.3
    col2=Part.makeCylinder(hd/2,hh+29.5,p+ax*(g+0.5),ax)
    print("    min dist to column (0.5 above seat):", round(s.distToShape(col2)[0],3))
