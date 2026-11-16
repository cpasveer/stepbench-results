from build123d import import_step
p=import_step('output.step')
es=sorted(p.edges(), key=lambda e:e.length)
for e in es[:6]: print(round(e.length,3), e.center())
print(len(p.faces()))
