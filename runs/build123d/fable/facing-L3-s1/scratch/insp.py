from build123d import *
for n in "AB":
    s=import_step(f"fixed_{n}.step"); b=s.bounding_box(); print(n,b.min,b.max,s.volume,len(s.faces()))
