import Part, json
R=json.load(open("task.json"))["poses"][0]["placements"]["A"]["rotation"]
import FreeCAD as App
lx=App.Vector(R[0][0],R[1][0],0); ly=App.Vector(R[0][1],R[1][1],0)
for n in ("output.step",):
    s=Part.read(n); print(n)
    for i,f in enumerate(s.Faces):
        t=f.Surface.__class__.__name__
        nn=f.normalAt(*[ (f.ParameterRange[0]+f.ParameterRange[1])/2,(f.ParameterRange[2]+f.ParameterRange[3])/2])
        print(i,t,round(f.Area,1),"n(lx,ly,z)=",round(nn.dot(lx),2),round(nn.dot(ly),2),round(nn.z,2))
