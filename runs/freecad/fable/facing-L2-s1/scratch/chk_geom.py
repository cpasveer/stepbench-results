import Part,FreeCAD as App
s=Part.read("output.step")
print(s.isValid(),len(s.Solids),s.BoundBox,len(s.Faces))
for n in ("fixed_A.step","fixed_B.step"):
    f=Part.read(n); print(n,f.BoundBox, s.common(f).Volume)
print(min(e.Length for e in s.Edges), min(f.Area for f in s.Faces))
for f in s.Faces:
    if f.Surface.__class__.__name__=="Plane" and abs(abs(f.normalAt(0,0).z)-1)<1e-6: print(f.Area, f.CenterOfMass.z, f.BoundBox.XLength,f.BoundBox.YLength)
