from build123d import *
import inspect
print([m for m in dir(Plane) if 'loc' in m.lower() or 'to' in m.lower()][:40])
print(inspect.signature(Location.__init__))
p = Plane(origin=(1,2,3), x_dir=(0,1,0), z_dir=(0,0,1))
print(p.x_dir, p.y_dir, p.z_dir)
s = Cylinder(10, 20)
print(type(s))
