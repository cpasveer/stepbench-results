import Part, FreeCAD as App
s=Part.read("output.step")
for i,f in enumerate(s.Faces):
    u,v=f.Surface.parameter(f.CenterOfMass) if hasattr(f.Surface,'parameter') else (0,0)
    try:
        n=f.normalAt(u,v)
    except Exception as e:
        continue
    if n.z<-0.5:
        bb=f.BoundBox
        print("D",i,round(f.Area,1),f.Surface.__class__.__name__,round(n.z,3),[round(x,1) for x in (bb.XMin,bb.YMin,bb.ZMin,bb.XMax,bb.YMax,bb.ZMax)])
