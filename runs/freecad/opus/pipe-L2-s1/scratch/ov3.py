import Part, FreeCAD as App
part=Part.read('output.step'); A=Part.read('fixed_A.step')
c=part.common(A)
s=c.Solids[0]
print(s.BoundBox)
print('inside A', A.isInside(s.BoundBox.Center,0.01,True), 'inside part', part.isInside(s.BoundBox.Center,0.01,True))
for f in s.Faces: print(type(f.Surface).__name__, round(f.Area,1))
