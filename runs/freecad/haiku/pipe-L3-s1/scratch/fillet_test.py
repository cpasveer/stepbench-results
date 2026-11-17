import math
import FreeCAD as App, Part
V=App.Vector
body=Part.read("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/output.step")
body=body.Solids[0]
def inside_angle(e):
    fs=body.ancestorsOfType(e,Part.Face)
    if len(fs)!=2: return None
    f1,f2=fs
    m=e.valueAt((e.FirstParameter+e.LastParameter)/2)
    n1=f1.normalAt(*f1.Surface.parameter(m)) if False else None
    try:
        n1=f1.normalAt(*f1.Surface.parameter(m)); n2=f2.normalAt(*f2.Surface.parameter(m))
    except Exception:
        return None
    # angle between outward normals; interior dihedral = 180 - angle between normals
    c=max(-1,min(1,n1.dot(n2)))
    ang=math.degrees(math.acos(c))
    return round(180-ang,1), round(e.Length,2)
res=[]
for e in body.Edges:
    a=inside_angle(e)
    if a and a[1]>0.5: res.append((a[0],a[1],[round(x,1) for x in e.CenterOfMass]))
res.sort()
print("edges with dihedral < 60 (count):", sum(1 for r in res if r[0]<60))
for r in res[:12]: print("edge", r)
sel=[]
for e in body.Edges:
    a=inside_angle(e)
    if a and a[0]<60 and a[1]>0.5: sel.append(e)
print("selected", len(sel))
for r in [2.5, 1.5]:
    try:
        f=body.makeFillet(r, sel)
        print("fillet ok r", r, "valid", f.isValid(), "solids", len(f.Solids), "vol", round(f.Volume,1))
        break
    except Exception as ex:
        print("fillet fail r", r, str(ex)[:120])
