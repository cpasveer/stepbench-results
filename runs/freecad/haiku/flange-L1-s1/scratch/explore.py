import json, os
import FreeCAD as App, Part
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
T = json.load(open(os.path.join(H, "task.json")))
P = T["poses"][0]["resolved"]
pl = T["poses"][0]["placements"]
def mat(R): return [list(r) for r in R]
def toA(v, RA, oA):
    d = [v[i]-oA[i] for i in range(3)]
    return [sum(RA[j][i]*d[j] for j in range(3)) for i in range(3)]
RA = pl["A"]["rotation"]; oA = pl["A"]["origin"]
RB = pl["B"]["rotation"]; oB = pl["B"]["origin"]
print("B origin in A-local", [round(x,3) for x in toA(oB,RA,oA)])
nB = [RB[0][2],RB[1][2],RB[2][2]]
print("B normal in A-local", [round(x,4) for x in toA([oB[i]+nB[i] for i in range(3)],RA,oA)] )
for b in P["bolts"]:
    print(b["id"], [round(x,2) for x in toA(b["position"],RA,oA)], "axis A-local", [round(x,3) for x in toA([b["axis"][i]+oA[i] for i in range(3)],RA,oA)])
for name in ["fixed_A.step","fixed_B.step"]:
    s = Part.read(os.path.join(H,name))
    bb = s.BoundBox
    print(name, "bbox", round(bb.XMin,2),round(bb.XMax,2),round(bb.YMin,2),round(bb.YMax,2),round(bb.ZMin,2),round(bb.ZMax,2), "vol", round(s.Volume,1), "solids", len(s.Solids))
    # project to A-local extents
    pts=[]
    for v in s.Vertexes:
        pts.append(toA([v.X,v.Y,v.Z],RA,oA))
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]
    print("   A-local extents x",round(min(xs),1),round(max(xs),1),"y",round(min(ys),1),round(max(ys),1),"z",round(min(zs),1),round(max(zs),1))
