import sys
sys.path.insert(0, "/home/user/stepbench-runs/tasks/pipe-L1-s1")
import submission as S
from build123d import Box, Plane
pad = S.pad_local().moved(S.PA.location)
plate = S.plate_local().moved(S.PB.location)
pad_top = S.PA.from_local_coords((0, 0, S.PIPE_R + S.PAD_T))
plate_pt = S.PB.from_local_coords((-8.0, 0.0, S.PLATE_T))
w = (pad_top - plate_pt).normalized()
start = plate_pt - w * 3.0; end = pad_top - w * 3.0
print("start", start, "end", end, "w", w)
span = (end - start).length; mid = (start + end) * 0.5
print("span", span, "mid", mid)
xdir = (S.PA.x_dir - w * S.PA.x_dir.dot(w)).normalized()
rib = Box(12.0, 8.0, span).moved(Plane(origin=mid, x_dir=xdir, z_dir=w).location)
print("rib bbox", rib.bounding_box().min, rib.bounding_box().max, rib.volume)
print("rib∩pad", (rib & pad).volume if (rib & pad) else None, "rib∩plate", (rib & plate).volume if (rib & plate) else None)
print("pad∩plate", (pad & plate))
print("pad solids", len(pad.solids()), "plate", len(plate.solids()))
