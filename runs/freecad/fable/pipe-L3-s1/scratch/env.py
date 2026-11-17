import sys
print(sys.version)
for m in ("numpy","scipy","scipy.sparse.linalg","gmsh","netgen","MeshPart","Fem","pypardiso","sksparse"):
    try:
        __import__(m); print(m,"ok")
    except Exception as e: print(m,"NO",e)
