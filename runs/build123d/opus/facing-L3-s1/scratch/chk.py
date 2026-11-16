from build123d import import_step
p=import_step('output.step'); A=import_step('fixed_A.step'); B=import_step('fixed_B.step')
print(p.bounding_box())
print('ovA',(p&A).volume,'ovB',(p&B).volume)
print('faces',len(p.faces()))
