from build123d import *
p=import_step('attempts/0002/output.step')
fs=p.faces()
print(len(fs))
pt=Vector(83.031,-77.892,31.481)
for i,f in enumerate(fs):
    d=f.distance_to(pt) if hasattr(f,'distance_to') else None
    if d is not None and d<0.5: print(i,f.geom_type,round(f.area,2),f.normal_at(pt) if True else '',d, f.bounding_box().min, f.bounding_box().max)
f=fs[0]; print(f.geom_type,f.area,f.bounding_box().min,f.bounding_box().max)
for e in f.edges(): print(e.geom_type, round(e.length,3), e.center())
