import sys
print(sys.version)
for m in ("Fem","femmesh.gmshtools","ObjectsFem","netgen","gmsh","numpy","scipy","scipy.sparse.linalg","MeshPart","Mesh"):
    try:
        __import__(m); print("ok",m)
    except Exception as e: print("no",m,type(e).__name__,str(e)[:80])
