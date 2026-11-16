from build123d import *
for n in "AB":
    s=import_step(f"fixed_{n}.step"); print(n, s.bounding_box())
