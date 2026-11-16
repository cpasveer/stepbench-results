from build123d import import_step
o=import_step('output.step')
for f in o.faces():
    if f.area<30: print('face',f.geom_type,round(f.area,3),f.center())
for e in o.edges():
    if e.length<0.5: print('edge',round(e.length,3),e.center())
