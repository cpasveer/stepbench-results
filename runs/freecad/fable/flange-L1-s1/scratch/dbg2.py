import json, sys, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); R=T["poses"][0]["resolved"]; B=R["bolts"]
s=Part.read("output.step")
for b in B:
    if b["id"] not in ("A1","A6","B1","B2"): continue
    p,ax=V(*b["position"]),V(*b["axis"])
    head=Part.makeCylinder(b["head_diameter"]/2+0.5,b["head_height"]+1,p+ax*10.01,ax)
    c=s.common(head)
    f=[m for m in R["mounting_faces"] if m["part"]==b["id"][0]][0]
    o=V(*f["origin"]); n=V(*f["normal"]); u=V(n.y,-n.x,0)
    for v in c.Vertexes[:12]:
        q=v.Point-o; print(b["id"], round(q.dot(n),2), round(q.dot(u),2), round(q.z,2), file=sys.stderr)
