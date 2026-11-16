import sys; sys.path.insert(0,'.')
from submission import *
a = math.radians(WEB_HALF_ANGLE)
sec = plane(P0 - 40 * AX, LY, AX) * Polygon((0, 0), (-80 * math.sin(a), 80 * math.cos(a)),(80 * math.sin(a), 80 * math.cos(a)), align=None)
w=extrude(sec,80); print(w.volume, w.bounding_box())
cyl=plane(P0, LY, AX) * Cylinder(R_PIPE + WEB_T, 100)
w2=w&cyl; print(w2.volume)
w3=w2 & (BED * Pos(0, 0, BAR_H / 2) * Box(BIG, BIG, BAR_H)); print(w3.volume)
