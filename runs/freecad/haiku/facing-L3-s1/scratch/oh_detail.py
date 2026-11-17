import traceback, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/oh_detail.txt", "w")
try:
    s = Part.read(D + "output.step").Solids[0]
    for name, d in [("+z", V(0,0,1)), ("+y", V(0,1,0)), ("+x", V(1,0,0))]:
        hmin = min(v.Point.dot(d) for v in s.Vertexes)
        rows = []
        for f in s.Faces:
            c = f.CenterOfMass
            if c.dot(d) - hmin < 0.05: continue
            u0,u1,v0,v1 = f.ParameterRange
            n = f.normalAt((u0+u1)/2, (v0+v1)/2)
            cosang = -n.dot(d)
            if cosang > 0.7071:
                rows.append((round(f.Area,1), f.Surface.TypeId.split("::")[-1], [round(x,1) for x in (c.x,c.y,c.z)]))
        rows.sort(reverse=True)
        out.write("%s total %d faces\n" % (name, len(rows)))
        for r in rows[:12]: out.write("   %s\n" % (r,))
except Exception:
    out.write(traceback.format_exc())
out.close()
