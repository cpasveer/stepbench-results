import json, os
import FreeCAD as App, Part
V = App.Vector
HERE="/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
task=json.load(open(os.path.join(HERE,"task.json")))
bolts={b["id"]:b for b in task["poses"][0]["resolved"]["bolts"]}
s=Part.read(os.path.join(HERE,"output.step"))
p=bolts["B1"]["position"]
band = Part.makeCylinder(4.75, 1.0, V(p[0],p[1],-33.5), V(0,0,1))
ov = s.common(band)
print("BAND total", round(ov.Volume,3))
for r0,r1 in [(0,2.75),(2.75,4.25),(4.25,4.75)]:
    ring = Part.makeCylinder(r1,1.0,V(p[0],p[1],-33.5),V(0,0,1)).cut(Part.makeCylinder(r0,1.0,V(p[0],p[1],-33.5),V(0,0,1))) if r0>0 else Part.makeCylinder(r1,1.0,V(p[0],p[1],-33.5),V(0,0,1))
    print("RING", r0, r1, round(ov.common(ring).Volume,3))
