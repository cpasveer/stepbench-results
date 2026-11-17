import json, math
import FreeCAD as App, Part
V=App.Vector
s=Part.read('output.step')
T=json.load(open('task.json'));R=T['poses'][0]['placements']['A']['rotation']; D=V(R[0][1],R[1][1],0); LX=V(R[0][0],R[1][0],0)
cA=V(-126,127.5,0)
for i,f in enumerate(s.Faces):
    c=f.CenterOfMass; n=f.normalAt(*f.Surface.parameter(c)) if f.Surface.__class__.__name__=='Plane' else None
    print(i, f.Surface.__class__.__name__, round(f.Area,3), 'loc', round((c-cA).dot(LX),2), round((c-cA).dot(D),2), round(c.z,2), 'nD', None if n is None else round(n.dot(D),3), 'nz', None if n is None else round(n.z,3))
