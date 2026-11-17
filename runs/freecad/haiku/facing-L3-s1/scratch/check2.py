import json, math, traceback, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/check2.txt", "w")
try:
    T = json.load(open(D + "task.json"))["poses"][0]["resolved"]
    part = Part.read(D + "output.step")
    fa = Part.read(D + "fixed_A.step"); fb = Part.read(D + "fixed_B.step")
    out.write("valid %s solids %d volume %.1f\n" % (part.isValid(), len(part.Solids), part.Volume))
    zA, zB = -94.0, -25.5
    for b in T["bolts"]:
        x, y, z = b["position"]; sg = b["axis"][2]; hd = b["head_diameter"]; hh = b["head_height"]
        hole = b["clearance_diameter_range"][0]
        pad_t = 4.0 if sg > 0 else 5.0
        # head bears on own pad: head base is pad far face
        base = zA + pad_t if sg > 0 else zB - pad_t
        # envelope: head + 0.5 radial, from base to head top + 1
        h0, h1 = sorted([base, base + sg * (hh + 1.0)])
        env = part.common(Part.makeCylinder(hd/2 + 0.5, h1-h0, V(x, y, h0), V(0,0,1))).Volume
        # tool cylinder 30 mm beyond head top
        t0 = base + sg * hh
        tz0, tz1 = sorted([t0, t0 + sg * 30.0])
        tool = part.common(Part.makeCylinder(hd/2, tz1-tz0, V(x, y, tz0), V(0,0,1))).Volume \
             + fa.common(Part.makeCylinder(hd/2, tz1-tz0, V(x, y, tz0), V(0,0,1))).Volume \
             + fb.common(Part.makeCylinder(hd/2, tz1-tz0, V(x, y, tz0), V(0,0,1))).Volume
        # channel: material inside the clearance hole over the whole gap, excluding its own pad
        ch = part.common(Part.makeCylinder(hole/2 + 0.1, zB - zA + 2, V(x, y, zA-1), V(0,0,1)))
        own_lo, own_hi = (zA, zA + pad_t) if sg > 0 else (zB - pad_t, zB)
        chv = 0.0
        for s_ in ch.Solids:
            cut = s_.cut(Part.makeBox(200, 200, own_hi - own_lo, V(x-100, y-100, own_lo)))
            chv += cut.Volume
        out.write("%s envelope %.3f tool %.3f channel_foreign %.3f\n" % (b["id"], env, tool, chv))
except Exception:
    out.write(traceback.format_exc())
out.close()
