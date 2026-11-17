import Part, itertools
s=Part.read("output.step")
pts=[v.Point for v in s.Vertexes]
d=sorted(((a-b).Length,i,j) for (i,a),(j,b) in itertools.combinations(enumerate(pts),2))
print("vertices",len(pts),"closest pairs",[round(x[0],3) for x in d[:6]])
print("shortest edges",sorted(round(e.Length,3) for e in s.Edges)[:6])
print("smallest faces",sorted(round(f.Area,2) for f in s.Faces)[:5])
