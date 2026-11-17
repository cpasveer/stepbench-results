import Part, json, FreeCAD as App
V=App.Vector
T=json.load(open("task.json"))["poses"][0]["resolved"]["mounting_faces"]
OA=V(*T[0]["origin"]);NA=V(*T[0]["normal"]);SA=V(NA.y,-NA.x,0)
OB=V(*T[1]["origin"]);NB=V(*T[1]["normal"]);UB=V(-NB.y,NB.x,0)
def fa(p): d=p-OA; return "A(x=%.2f s=%.2f z=%.2f)"%(d.dot(NA),d.dot(SA),p.z)
def fb(p): d=p-OB; return "B(w=%.2f u=%.2f z=%.2f)"%(d.dot(NB),d.dot(UB),p.z)
for n,idx,fr in (("0001",[0,1,2,31,32,33],fa),("0002",[29,30,31,37,38,39,43,44,45],fb)):
    s=Part.read("attempts/%s/output.step"%n)
    for i in idx:
        f=s.Faces[i]
        print(n,i,f.Surface.__class__.__name__,round(f.Area,2),"n=",[round(c,2) for c in f.normalAt(0,0)] if hasattr(f,'normalAt') else '')
        print("    ",[fr(v.Point) for v in f.Vertexes])
