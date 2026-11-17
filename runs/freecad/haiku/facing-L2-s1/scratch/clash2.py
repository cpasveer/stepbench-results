import json, os
import FreeCAD as App, Part
V = App.Vector
HERE="/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
task=json.load(open(os.path.join(HERE,"task.json")))
bolts={b["id"]:b for b in task["poses"][0]["resolved"]["bolts"]}
s=Part.read(os.path.join(HERE,"output.step"))
B_bear = -33.5
for bid in ["B1","B2","B3"]:
    b=bolts[bid]; p=b["position"]
    hr = b["head_diameter"]/2 + 0.5
    # head lies from the bearing plane outward (away from the fixed part, i.e. -Z for B), plus 1 mm beyond its top
    env = Part.makeCylinder(hr, b["head_height"] + 1.0, V(p[0], p[1], B_bear - b["head_height"] - 1.0), V(0,0,1))
    ov = s.common(env)
    if ov.Volume > 1e-6:
        bb=ov.BoundBox
        print("CLASH", bid, round(ov.Volume,3), "z", round(bb.ZMin,2), round(bb.ZMax,2), "x", round(bb.XMin,2), round(bb.XMax,2), "y", round(bb.YMin,2), round(bb.YMax,2))
    else:
        print("ok", bid)
