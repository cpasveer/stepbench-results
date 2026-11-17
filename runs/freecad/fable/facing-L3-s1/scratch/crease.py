import sys, os, math, Part, FreeCAD as App
f=os.environ.get("CR_STEP","output.step")
s=Part.read(f)
faces=s.Faces
emap={}
for i,fa in enumerate(faces):
    for e in fa.Edges:
        k=e.hashCode()
        emap.setdefault(k,[]).append((i,e))
res=[]
for k,lst in emap.items():
    # group same edges
    for a in range(len(lst)):
        for b in range(a+1,len(lst)):
            (i,e1),(j,e2)=lst[a],lst[b]
            if i==j or not e1.isSame(e2): continue
            pm=e1.valueAt((e1.FirstParameter+e1.LastParameter)/2)
            try:
                n1=faces[i].normalAt(*faces[i].Surface.parameter(pm)); n2=faces[j].normalAt(*faces[j].Surface.parameter(pm))
            except Exception: continue
            ang=math.degrees(n1.getAngle(n2))
            res.append((ang,i,j,pm,e1.Length,faces[i].Surface.__class__.__name__,faces[j].Surface.__class__.__name__))
res.sort(key=lambda r:r[0])
print(f,"edges checked",len(res))
for r in res[:6]: print("  dihedral-normal angle %.3f deg faces %d,%d at %s len %.2f %s %s"%(r[0],r[1],r[2],[round(v,2) for v in r[3]],r[4],r[5],r[6]))
