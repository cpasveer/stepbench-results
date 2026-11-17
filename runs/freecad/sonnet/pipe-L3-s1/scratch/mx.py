import Part
s=Part.read('output.step').Solids[0]
a=sorted([round(f.Area) for f in s.Faces if f.Surface.__class__.__name__=='Plane'],reverse=True)[:8]
print('top planar areas',a)
