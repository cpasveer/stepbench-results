import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
b = V(0.927, 0.369, -0.069); b.normalize()
rows=[]
for f in s.Faces:
    u0,u1,v0,v1=f.ParameterRange
    n = f.normalAt((u0+u1)/2,(v0+v1)/2)
    if n.dot(b) < -0.7071:
        c = f.CenterOfMass
        rows.append((round(f.Area,1), [round(x,1) for x in c], f.Surface.__class__.__name__, [round(x,2) for x in n]))
rows.sort(reverse=True)
for r in rows[:14]: print("OVH", r)
print("count", len(rows))
