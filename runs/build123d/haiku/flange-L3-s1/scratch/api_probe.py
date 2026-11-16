from build123d import *
import build123d, inspect
print(build123d.__version__)
p = Plane(origin=(1,2,3), x_dir=(1,0,0), z_dir=(0,0,1))
print(type(p.location), p.location)
b = Box(2,3,4, align=(Align.CENTER, Align.CENTER, Align.MIN))
print(b.bounding_box().min, b.bounding_box().max)
c = Cylinder(1, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
print(c.bounding_box().min, c.bounding_box().max)
print(inspect.signature(Solid.extrude))
f = Face(Wire.make_polygon([Vector(0,0,0),Vector(1,0,0),Vector(1,1,0)]))
print(Solid.extrude(f, Vector(0,0,2)).volume)
print(b.moved(p.location).bounding_box().min)
