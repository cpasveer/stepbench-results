import json, traceback, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/probe.txt", "w")
try:
    part = Part.read(D + "output.step")
    T = json.load(open(D + "task.json"))["poses"][0]["resolved"]
    for b in T["bolts"]:
        x, y, z = b["position"]
        col = Part.makeCylinder(2.25, 80, V(x, y, -100), V(0, 0, 1))
        com = part.common(col)
        zs = sorted([round(s.BoundBox.ZMin, 2) for s in com.Solids] + [round(s.BoundBox.ZMax, 2) for s in com.Solids])
        out.write("%s column-material-volume %.2f  z-ranges %s\n" % (b["id"], com.Volume, zs))
    out.write("part bbox %s\n" % part.BoundBox)
    # slices at z levels to see what is there at A3 radius 8
    x, y = T["bolts"][2]["position"][:2]
    for zz in [-90, -85, -70, -50, -35, -28]:
        sl = part.common(Part.makeCylinder(8, 0.2, V(x, y, zz), V(0, 0, 1)))
        out.write("z %.1f within r8 of A3: vol %.2f\n" % (zz, sl.Volume))
except Exception:
    out.write(traceback.format_exc())
out.close()
