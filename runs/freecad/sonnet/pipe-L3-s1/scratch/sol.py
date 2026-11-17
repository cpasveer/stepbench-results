import Part
s=Part.read('scratch/v.step')
for i,x in enumerate(s.Solids): print(i,round(x.Volume),x.BoundBox)
