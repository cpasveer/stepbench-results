import json, os
import FreeCAD as App, Part
V = App.Vector
HERE="/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
task=json.load(open(os.path.join(HERE,"task.json")))
bolts={b["id"]:b for b in task["poses"][0]["resolved"]["bolts"]}
s=Part.read(os.path.join(HERE,"output.step"))
B_bear=-33.5
tot = {}
for name, fn in {
  "shank r2.5 z[-33.5,-25.5]": lambda p: Part.makeCylinder(2.5, 8.0, V(p[0],p[1],-33.5), V(0,0,1)),
  "shank r2.75 z[-33.5,-25.5]": lambda p: Part.makeCylinder(2.75, 8.0, V(p[0],p[1],-33.5), V(0,0,1)),
  "band r4.75 z[-33.5,-32.5]": lambda p: Part.makeCylinder(4.75, 1.0, V(p[0],p[1],-33.5), V(0,0,1)),
  "band r4.75 z[-34.5,-33.5]": lambda p: Part.makeCylinder(4.75, 1.0, V(p[0],p[1],-34.5), V(0,0,1)),
  "envelope r4.75 z[-39.5,-33.5] (mine)": lambda p: Part.makeCylinder(4.75, 6.0, V(p[0],p[1],-39.5), V(0,0,1)),
}.items():
    v=0
    for bid in ["B1","B2","B3"]:
        p=bolts[bid]["position"]
        v += s.common(fn(p)).Volume
    print("CAND", name, round(v,3))
