import sys; sys.path.insert(0,'.')
from submission import *
p=build()
fs=p.faces(); print(len(fs), len(p.edges()))
for i,f in enumerate(fs):
    if f.area<12: print(i,f.geom_type,round(f.area,3),f.center())
es=[e for e in p.edges() if e.length<0.6]
for e in es: print('edge',round(e.length,3),e.center())
