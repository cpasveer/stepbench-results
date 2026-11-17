import os
import Part
s = Part.read("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/output.step")
for e in s.Edges:
    if e.Length < 0.5:
        print("SHORT edge len %.3f at %s curve %s" % (e.Length, tuple(round(c,2) for c in e.CenterOfMass), e.Curve.__class__.__name__))
for f in s.Faces:
    if f.Area < 3:
        print("SMALL face area %.3f at %s" % (f.Area, tuple(round(c,2) for c in f.CenterOfMass)))
