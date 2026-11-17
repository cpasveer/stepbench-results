import FreeCAD as App, Part, Fem, ObjectsFem, time
doc=App.newDocument("m")
s=Part.read("attempts/0002/output.step")
o=doc.addObject("Part::Feature","p"); o.Shape=s
try:
    m=ObjectsFem.makeMeshNetgenLegacy(doc,"mesh")
    m.Shape=o; m.MaxSize=4.0; m.MinSize=0.5; m.SecondOrder=True; m.Fineness="Moderate"
    t=time.time(); doc.recompute()
    fm=m.FemMesh
    print("netgen nodes",fm.NodeCount,"vol elems",fm.VolumeCount,"t",time.time()-t)
except Exception as e:
    print("netgen legacy fail",type(e).__name__,e)
print([n for n in dir(ObjectsFem) if "esh" in n])
