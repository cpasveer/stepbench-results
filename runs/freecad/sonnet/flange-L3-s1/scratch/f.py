import Part
s=Part.read("output.step")
for i,f in enumerate(s.Faces):
    bb=f.BoundBox
    if f.Area<60: print(i,round(f.Area,3),f.Surface.__class__.__name__,[round(x,2) for x in (bb.XMin,bb.YMin,bb.ZMin,bb.XMax,bb.YMax,bb.ZMax)])
