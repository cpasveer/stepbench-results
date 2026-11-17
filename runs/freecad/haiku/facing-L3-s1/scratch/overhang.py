import traceback, sys, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/overhang.txt", "w")
def analyse(shape, name):
    dirs = {"+z": V(0,0,1), "-z": V(0,0,-1), "+x": V(1,0,0), "-x": V(-1,0,0), "+y": V(0,1,0), "-y": V(0,-1,0)}
    for k, d in dirs.items():
        # coordinate along build direction: height h = p.dot(d); bed = min h
        hs = [v.Point.dot(d) for v in shape.Vertexes]
        hmin = min(hs)
        oh = 0.0
        for f in shape.Faces:
            c = f.CenterOfMass
            if c.dot(d) - hmin < 0.05:
                continue  # on bed
            u0, u1, v0, v1 = f.ParameterRange
            n = f.normalAt((u0+u1)/2, (v0+v1)/2)
            # downward-facing relative to build dir: normal opposite to d, steeper than 45 deg
            cosang = -n.dot(d)
            if cosang > 0.7071:
                oh += f.Area * (1.0 if cosang > 0 else 0)
        out.write("%s %s overhang_area %.1f\n" % (name, k, oh))
try:
    s = Part.read(D + "output.step").Solids[0]
    analyse(s, "design")
except Exception:
    out.write(traceback.format_exc())
out.close()
