from build123d import *
ta=Vector(-0.6743023875837234,0.7384553406258838,0)
pl=Plane(origin=(95,-91,5.5), x_dir=ta, z_dir=(0,0,1))
print(pl.x_dir,pl.y_dir,pl.location)
b=Pos(0,10,0)*Box(1,1,1)
print((pl.location*b).center(), pl.from_local_coords(Vector(0,10,0)))
