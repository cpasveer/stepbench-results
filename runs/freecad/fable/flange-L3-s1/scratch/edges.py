import Part, json, FreeCAD as App
V=App.Vector
T=json.load(open("task.json"))["poses"][0]["resolved"]["mounting_faces"]
OA=V(*T[0]["origin"]);NA=V(*T[0]["normal"]);SA=V(NA.y,-NA.x,0)
OB=V(*T[1]["origin"]);NB=V(*T[1]["normal"]);UB=V(-NB.y,NB.x,0)
def fa(p): d=p-OA; return "A(x=%.2f s=%.2f z=%.2f)"%(d.dot(NA),d.dot(SA),p.z)
def fb(p): d=p-OB; return "B(w=%.2f u=%.2f z=%.2f)"%(d.dot(NB),d.dot(UB),p.z)
s=Part.read("output.step")
for e in s.Edges:
    if e.Length<1.5: print(round(e.Length,3),[fa(v.Point)+" "+fb(v.Point) for v in e.Vertexes])
for i,f in enumerate(s.Faces):
    if f.Area<8: print("face",i,round(f.Area,3),[fb(v.Point) for v in f.Vertexes])
