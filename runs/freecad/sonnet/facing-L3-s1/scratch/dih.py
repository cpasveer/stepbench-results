import Part, math
s=Part.read("output.step")
emap={}
for fi,f in enumerate(s.Faces):
    for e in f.Edges:
        for ei,e2 in enumerate(s.Edges):
            if e.isSame(e2): emap.setdefault(ei,[]).append(fi)
for ei,fs in emap.items():
    if len(fs)!=2: continue
    e=s.Edges[ei]
    f1,f2=s.Faces[fs[0]],s.Faces[fs[1]]
    p=e.valueAt((e.FirstParameter+e.LastParameter)/2)
    try:
        n1=f1.normalAt(*f1.Surface.parameter(p)); n2=f2.normalAt(*f2.Surface.parameter(p))
    except Exception: continue
    a=math.degrees(math.acos(max(-1,min(1,n1.dot(n2)))))
    inside=180-a
    if 0.5<abs(a)<=70 and e.Length>0.5 or inside<45 and a>110:
        print(round(inside,1),[round(c,1) for c in p],round(e.Length,1),f1.Surface.__class__.__name__,f2.Surface.__class__.__name__)
