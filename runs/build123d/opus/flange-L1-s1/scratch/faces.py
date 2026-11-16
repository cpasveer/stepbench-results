from build123d import *
import sys; sys.path.insert(0,'.')
import submission as s
p=s.build()
print(len(p.faces()), [f.geom_type for f in p.faces()])
