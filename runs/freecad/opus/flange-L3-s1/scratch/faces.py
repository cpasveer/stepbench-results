import Part, FreeCAD as App
s=Part.read("output.step")
for i,f in enumerate(s.Faces):
    t=f.Surface.__class__.__name__
    L=max(e.Length for e in f.Edges)
    w=f.Area/L
    if w<7 and t=="Plane":
        u0,u1,v0,v1=f.ParameterRange
        n=f.normalAt((u0+u1)/2,(v0+v1)/2)
        print(i,t,"area",round(f.Area,1),"approx width",round(w,2),"n",[round(x,2) for x in n],"c",[round(x,1) for x in f.CenterOfMass])
