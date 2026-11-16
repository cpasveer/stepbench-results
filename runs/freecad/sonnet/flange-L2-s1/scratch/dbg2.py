import Part
s=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L2-s1/output.step")
L=[]
for i,f in enumerate(s.Faces):
  c=f.Surface.__class__.__name__
  if c=="Plane":
    n=f.normalAt(0,0); p=f.CenterOfMass
    L.append((i,c,tuple(round(x,3) for x in n),round(n.dot(p),3),round(f.Area,1)))
  else:
    L.append((i,c,round(f.Area,1), str(f.Surface.Radius) if hasattr(f.Surface,'Radius') else ''))
for x in L: print("F",x)
