import Part,FreeCAD as App
for f in ['fixed_A.step','fixed_B.step']:
    s=Part.read(f); print(f,s.BoundBox,len(s.Solids),len(s.Faces))
    for i,fc in enumerate(s.Faces[:40]): print(' ',i,fc.Surface.__class__.__name__,round(fc.Area,1),fc.CenterOfMass)
