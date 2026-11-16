from build123d import import_step
p=import_step("output.step")
for f in ["fixed_A.step","fixed_B.step"]:
    s=import_step(f)
    r=p&s; print(f,"overlap", 0 if r is None else r.volume)
print(p.bounding_box().min,p.bounding_box().max, len(p.faces()))
