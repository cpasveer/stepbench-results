import sys; sys.path.insert(0,'.')
from submission import *
p=build()
for s in p.solids(): print(s.volume, s.bounding_box().min, s.bounding_box().max)
oA,nA,tA=frame("A"); oB,nB,tB=frame("B")
pA=oA+tA*UA; pB=oB+tB*UB
bm=beam(pA,pB,oA)
print([ (s.volume) for s in bm.solids()], bm.bounding_box().min, bm.bounding_box().max)
