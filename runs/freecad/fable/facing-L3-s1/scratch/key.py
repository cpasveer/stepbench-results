import json, math, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); P=T["poses"][0]
RA=P["placements"]["A"]["rotation"]
s=Part.read("output.step")
b=[b for b in P["resolved"]["bolts"] if b["id"]=="B1"][0]
p=V(*b["position"]); a=V(*b["axis"])
for depth in (14,17,20,23,26,29,32,35,38):   # leg centre below flange underside
    free=[]
    for k in range(120):
        ang=math.radians(k*3); dl=V(RA[0][0]*math.cos(ang)+RA[0][1]*math.sin(ang),RA[1][0]*math.cos(ang)+RA[1][1]*math.sin(ang),0)
        leg=Part.makeCylinder(2.4,70,p+a*(3.9+depth),dl)
        free.append(s.distToShape(leg)[0]>0.3)
    best=run=0
    for v in free+free:
        run=run+1 if v else 0; best=max(best,run)
    print("leg centre z local",68.5-3.9-depth,"sweep",min(best,120)*3)
