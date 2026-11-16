import sys
sys.path.insert(0, "/home/user/stepbench-runs/tasks/pipe-L1-s1")
import submission as S
from build123d import Vector
pad = S.pad_local().moved(S.PA.location)
plate = S.plate_local().moved(S.PB.location)
for n, s in (("pad", pad), ("plate", plate)):
    print(n, len(s.solids()), s.bounding_box().min, s.bounding_box().max, s.volume)
print("PB origin", S.PB.origin, "PB x", S.PB.x_dir, "z", S.PB.z_dir)
print("plate top pt", S.PB.from_local_coords((-8,0,6)))
print("pad top", S.PA.from_local_coords((0,0,42)))
print("PA origin", S.PA.origin, S.PA.x_dir, S.PA.z_dir)
