import sys; sys.path.insert(0,'.')
import submission as s
from submission import *
pipe = Rot(0, 90, 0) * Cylinder(R_PIPE, 400)
for e in [12,8,4,0,-2,-4,-6,-8,-10]:
    s.ARM_EXT=e
    b=(s.saddle().fuse(s.plate(),s.arm()).clean())-pipe
    print(e, round(b.volume,1), len(b.faces()))
