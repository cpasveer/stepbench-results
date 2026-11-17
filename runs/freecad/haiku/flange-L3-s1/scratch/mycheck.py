import json, os, math
import FreeCAD as App, Part
V = App.Vector
GRIP = 8.0
D = "/home/user/stepbench-runs/tasks-freecad/flange-L3-s1"
T = json.load(open(D + "/task.json"))
R = T["poses"][0]["resolved"]
out = Part.read(D + "/output.step")
fa = Part.read(D + "/fixed_A.step"); fb = Part.read(D + "/fixed_B.step")
print("valid", out.isValid(), "solids", len(out.Solids), "vol %.1f" % out.Volume)
print("overlap A %.4f  B %.4f" % (out.common(fa).Volume, out.common(fb).Volume))
def cyl_free(p, n, r, h0, h1):
    c = Part.makeCylinder(r, h1 - h0, p + n * h0, n)
    return out.common(c).Volume, fa.common(c).Volume + fb.common(c).Volume
for b in R["bolts"]:
    p = V(*b["position"]); n = V(*b["axis"])
    hd = b["head_diameter"]; hh = b["head_height"]; g = b["grip_range"]
    # head envelope: radius hd/2+0.5, from grip to grip+hh+1 ; grip assumed 5 (our boss)
    env = cyl_free(p, n, hd / 2 + 0.5, GRIP, GRIP + hh + 1.0)
    # tool access: radius hd/2 from head top, 30 mm
    tool = cyl_free(p, n, hd / 2, GRIP + hh, GRIP + hh + 30.0)
    print(b["id"], "head-env overlap (mm3) %.3f" % env[0], "tool part %.3f fixed %.3f" % (tool[0], tool[1]))
