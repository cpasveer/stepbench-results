import json, os
import FreeCAD as App, Part
V = App.Vector
HERE="/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
task=json.load(open(os.path.join(HERE,"task.json")))
bolts={b["id"]:b for b in task["poses"][0]["resolved"]["bolts"]}
s=Part.read(os.path.join(HERE,"output.step"))
for bid in ["B1","B2","B3","A1","A2","A3","A4"]:
    b=bolts[bid]; p=V(*b["position"]); ax=V(*b["axis"])
    hr = b["head_diameter"]/2 + 0.5
    ha = b["head_height"]
    # envelope: head zone plus 1 mm beyond the head top (along axis away from fixed part)
    env = Part.makeCylinder(hr, ha + 1.0, p, ax)  # starts at the bearing plane and extends along the axis (into head region)
    ov = s.common(env)
    if ov.Volume > 1e-6:
        bb=ov.BoundBox
        print("CLASH", bid, round(ov.Volume,3), "bbox z", round(bb.ZMin,2), round(bb.ZMax,2), "x", round(bb.XMin,2), round(bb.XMax,2), "y", round(bb.YMin,2), round(bb.YMax,2))
    else:
        print("ok", bid)
