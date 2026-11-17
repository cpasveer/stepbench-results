import Part, json, FreeCAD as App
V=App.Vector
T=json.load(open("task.json"))["poses"][0]["resolved"]["mounting_faces"]
for nm,f in zip("AB",T):
    s=Part.read("fixed_%s.step"%nm)
    o=V(*f["origin"]); n=V(*f["normal"]); t=V(n.y,-n.x,0)
    print(nm, len(s.Solids), s.Volume, s.BoundBox)
    ws=[(v.Point-o).dot(n) for v in s.Vertexes]; ts=[(v.Point-o).dot(t) for v in s.Vertexes]; zs=[v.Point.z-o.z for v in s.Vertexes]
    print(" n range",min(ws),max(ws)," t range",min(ts),max(ts)," z",min(zs),max(zs))
    for fc in s.Faces:
        print("  ",fc.Surface.__class__.__name__, round(fc.Area,1), [round((fc.CenterOfMass-o).dot(n),2), round((fc.CenterOfMass-o).dot(t),2), round(fc.CenterOfMass.z-o.z,2)])
