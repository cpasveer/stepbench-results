import Part, FreeCAD as App
import numpy as np
nA=App.Vector(0.7384553406258838,0.6743023875837234,0)
nB=App.Vector(-0.4786918579404784,-0.8779829754276512,0)
for f,n,o in [("fixed_A.step",nA,App.Vector(95,-91,5.5)),("fixed_B.step",nB,App.Vector(132.06606660467747,-30.747558502102073,-18))]:
    s=Part.read(f)
    print(f, s.BoundBox, len(s.Solids), s.Volume, len(s.Faces))
    # extent in local coordinates
    u=App.Vector(-n.y,n.x,0); z=App.Vector(0,0,1)
    pts=[v.Point for v in s.Vertexes]
    for name,ax in [("n",n),("u",u),("z",z)]:
        d=[(p-o).dot(ax) for p in pts]; print(name,min(d),max(d))
    for fc in s.Faces:
        print("  ",fc.Surface.__class__.__name__, round(fc.Area,1), [round(x,2) for x in [(fc.CenterOfMass-o).dot(n),(fc.CenterOfMass-o).dot(u),(fc.CenterOfMass-o).dot(z)]])
