import FreeCAD as App, Part
s=Part.read('/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1/output.step')
print('valid',s.isValid(),'solids',len(s.Solids),'vol',s.Volume)
for n in 'AB':
    f=Part.read(f'/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1/fixed_{n}.step')
    print(n,'common',s.common(f).Volume)
