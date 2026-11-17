import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
b = V(0.248796, 0.590065, 0.768065); b.normalize()
rows=[]
for f in s.Faces:
    u0,u1,v0,v1=f.ParameterRange
    n = f.normalAt((u0+u1)/2,(v0+v1)/2)
    d = -n.dot(b)           # cos of normal angle from straight down
    if d > 0.5:
        nfd = round(__import__('math').degrees(__import__('math').acos(max(-1,min(1,d)))),1)
        rows.append((nfd, round(f.Area,1), [round(x,1) for x in f.CenterOfMass], f.Surface.__class__.__name__, [round(x,2) for x in n]))
rows.sort()
for r in rows: print("DOWN", r)
