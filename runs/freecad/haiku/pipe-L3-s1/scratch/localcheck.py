import json, os
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
POSE = T["poses"][0]["resolved"]
body = Part.read(os.path.join(HERE, "output.step"))
print("solids", len(body.Solids), "valid", body.isValid(), "volume %.1f" % body.Volume, "faces", len(body.Faces))
fa = Part.read(os.path.join(HERE, "fixed_A.step"))
fb = Part.read(os.path.join(HERE, "fixed_B.step"))
print("overlap A %.4f  B %.4f" % (body.common(fa).Volume, body.common(fb).Volume))
dA = body.distToShape(fa)[0]; dB = body.distToShape(fb)[0]
print("min dist to A %.3f  to B %.3f" % (dA, dB))
HEAD = {"M5": (8.5, 5.0), "M4": (7.0, 4.0)}
for b in POSE["bolts"]:
    pos = V(*b["position"]); ax = V(*b["axis"]); hd, hh = HEAD[b["size"]]
    # seat: where body's bearing face is: find by grip = distance along axis to body top
    # probe along the axis for material near the bolt
    ts = [i * 0.1 for i in range(0, 300)]
    inside = []
    for t in ts:
        p = pos + ax * t
        # is the point inside body?
        if body.isInside(p, 1e-6, True):
            inside.append(t)
    seat = 5.0 if b['size']=='M5' and b['id'].startswith('A') else (4.0 if b['id'].startswith('B') else 5.0)
    # head clearance cylinder above the seat
    seat_pt = pos + ax * seat
    hc = Part.makeCylinder(hd / 2 + 0.5, hh + 1.0, seat_pt, ax)
    tc = Part.makeCylinder(hd / 2, 30.0, pos + ax * (seat + hh), ax)
    print(b["id"], "last material along axis at t=%.2f" % seat,
          "head-clear overlap %.4f" % body.common(hc).Volume,
          "tool-cyl overlap %.4f" % body.common(tc).Volume,
          "tool-cyl vs A/B %.4f %.4f" % (fa.common(tc).Volume, fb.common(tc).Volume))
