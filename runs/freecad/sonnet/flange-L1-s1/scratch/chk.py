import Part
s=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L1-s1/output.step");print("RES",s.isValid(),len(s.Solids),s.Volume,s.BoundBox)
for n in "AB":
    f=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L1-s1/fixed_%s.step"%n);print("RES",n,s.common(f).Volume)
