from build123d import *
p=import_step("output.step")
for n in "AB":
    s=import_step(f"fixed_{n}.step"); print(n,"overlap",(p&s).volume if (p&s) else 0)
print(len(p.faces()), sorted(round(f.area,2) for f in p.faces())[:5], min(e.length for e in p.edges()))
for f in p.faces(): print(f.geom_type, round(f.area,1), [round(v,1) for v in f.center()])
