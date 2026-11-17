import math
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
print("faces", len(s.Faces), "edges", len(s.Edges))
print("small edges (<0.1):", [round(e.Length,3) for e in s.Edges if e.Length < 0.1])
print("small faces (<0.1 mm2):", [round(f.Area,4) for f in s.Faces if f.Area < 0.1])
print("sliver-ish faces (<0.25 mm wide?):", [round(f.Area,3) for f in s.Faces if f.Area < 1.0])
# dihedral angles between adjacent planar faces
def nrm(f):
    u0,u1,v0,v1=f.ParameterRange
    return f.normalAt((u0+u1)/2,(v0+v1)/2)
bad=[]
for e in s.Edges:
    fs = s.ancestorsOfType(e, Part.Face)
    if len(fs)!=2: continue
    f1,f2 = fs
    if f1.Surface.__class__.__name__!="Plane" or f2.Surface.__class__.__name__!="Plane": continue
    n1=nrm(f1); n2=nrm(f2)
    ang = math.degrees(math.acos(max(-1,min(1,n1.dot(n2)))))  # angle between normals
    # interior dihedral angle = 180 - ang for convex, 180+ang for concave; report folds
    if ang < 2.0: bad.append(("near-coplanar crease", round(ang,2), round(e.Length,2)))
    interior = 180 - ang
    if interior < 45 or interior > 175 and ang>2:
        bad.append(("shallow/knife", round(interior,1), round(e.Length,2)))
print("crease/shallow flags:", len(bad), bad[:20])
