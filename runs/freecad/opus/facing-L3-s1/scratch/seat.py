import FreeCAD as App, Part
V=App.Vector
s=Part.read('output.step'); cB=V(-124.18164228311065,135.11206773573574,0)
for f in s.Faces:
    if f.Surface.__class__.__name__=='Plane' and abs(f.CenterOfMass.z+25.5)<1e-6:
        m=0
        for e in f.Edges:
            for p in e.discretize(200):
                m=max(m,(V(p.x,p.y,0)-cB).Length)
        print('B seat max r',m)
