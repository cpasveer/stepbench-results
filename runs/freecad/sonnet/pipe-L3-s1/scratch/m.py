import FreeCAD as App, Part
print([m for m in ("Fem","femmesh","Mesh","MeshPart") ])
try:
    import Fem; print('Fem ok')
except Exception as e: print('no Fem',e)
try:
    import netgen; print('netgen py ok')
except Exception as e: print('no netgen py',e)
import shutil
print(shutil.which('gmsh'),shutil.which('netgen'))
try:
    import Fem
    s=Part.makeBox(10,10,10)
    m=Fem.FemMesh()
    print([x for x in dir(Fem) if 'esh' in x or 'Gmsh' in x or 'Netgen' in x])
except Exception as e: print(e)
