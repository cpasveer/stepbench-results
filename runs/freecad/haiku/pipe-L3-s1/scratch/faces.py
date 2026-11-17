import json, os
import FreeCAD as App, Part
V=App.Vector
HERE="/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1"
T=json.load(open(os.path.join(HERE,"task.json")))
CB=V(*T["poses"][0]["placements"]["B"]["origin"])
NB=V(*T["poses"][0]["resolved"]["mounting_faces"][0]["normal"]); NB.normalize()
UB=V(0.9214414224885092,-0.3885173161164798,0); ZB=V(0,0,1)
body=Part.read(os.path.join(HERE,"output.step"))
p=V(19.626,57.052,54.721)
best=None
for i,f in enumerate(body.Faces):
    d=f.distToShape(Part.Vertex(p))[0]
    if best is None or d<best[0]: best=(d,i,f)
d,i,f=best
u,v=f.Surface.parameter(p) if hasattr(f.Surface,'parameter') else (0,0)
n=f.normalAt(*f.Surface.parameter(p)) if False else None
try:
    uv=f.Surface.parameter(p)
    n=f.normalAt(*uv)
except Exception as e:
    c=f.CenterOfMass; n=f.normalAt(*f.Surface.parameter(c))
print("face",i,"type",f.Surface.__class__.__name__,"dist",round(d,3),"area",round(f.Area,1),"normal",[round(x,3) for x in n], "normal·(-NB)",round(n.dot(-NB),3))
print("uvn of point", round(((p-CB).dot(UB)),2), round(((p-CB).dot(ZB)),2), round(((p-CB).dot(NB)),2))
for j,f2 in enumerate(body.Faces):
    if f2.Area>1.0 and f2.BoundBox.ZMin < 1e9:
        try:
            c=f2.CenterOfMass; n2=f2.normalAt(*f2.Surface.parameter(c))
        except Exception: n2=None
        if n2 is not None and n2.dot(-NB)>0.6:
            print("downward-ish face", j, f2.Surface.__class__.__name__, "area", round(f2.Area,1), "n·(-NB)", round(n2.dot(-NB),3), "centre uvn", [round(x,1) for x in ((c-CB).dot(UB),(c-CB).dot(ZB),(c-CB).dot(NB))])
