import Part
s=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L2-s1/output.step")
print("SOL",len(s.Solids),s.Volume,len(s.Faces),len(s.Vertexes),s.isValid())
vs=[v.Point for v in s.Vertexes]
for i in range(len(vs)):
  for j in range(i):
    if vs[i].distanceToPoint(vs[j])<0.3: print("NEAR",vs[i],vs[j])
for f in s.Faces:
  print(f.Surface.__class__.__name__, round(f.Area,1))
