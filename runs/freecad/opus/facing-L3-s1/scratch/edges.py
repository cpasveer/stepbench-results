import FreeCAD as App, Part, math
V=App.Vector
s=Part.read('output.step').Solids[0]
anc={}
for i,f in enumerate(s.Faces):
    for e in f.Edges: anc.setdefault(e.hashCode(),[]).append(i)
for e in s.Edges:
    fs=anc.get(e.hashCode(),[])
    if len(fs)!=2: continue
    worst=999
    for k in range(1,6):
        p=e.valueAt(e.FirstParameter+(e.LastParameter-e.FirstParameter)*k/6)
        ns=[]
        for i in fs:
            f=s.Faces[i]; u,v=f.Surface.parameter(p); ns.append(f.normalAt(u,v))
        # tangent
        t=e.tangentAt(e.FirstParameter+(e.LastParameter-e.FirstParameter)*k/6)
        # material interior angle: use offset test
        a=math.degrees(ns[0].getAngle(ns[1]))
        # convex if moving along bisector of -normals stays inside
        b=-(ns[0]+ns[1]); 
        if b.Length<1e-9: continue
        b.normalize(); conv=s.isInside(p+b*0.05,1e-5,False)
        interior=180-a if conv else 180+a
        worst=min(worst,interior)
    if worst<75: print('edge interior %.1f'%worst, fs, [round(c,1) for c in e.CenterOfMass], s.Faces[fs[0]].Surface.__class__.__name__, s.Faces[fs[1]].Surface.__class__.__name__)
