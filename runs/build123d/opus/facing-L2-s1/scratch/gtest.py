from build123d import *
tri = Polygon((0, 0), (5, -5), (-5, -5), align=None)
g=extrude(Plane.YZ * tri, 10); print(g.bounding_box())
