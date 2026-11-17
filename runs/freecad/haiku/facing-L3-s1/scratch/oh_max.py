import traceback, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/oh_max.txt", "w")
try:
    s = Part.read(D + "output.step").Solids[0]
    d = V(0,0,1); zmin = min(v.Point.z for v in s.Vertexes)
    rows = []
    for f in s.Faces:
        u0,u1,v0,v1 = f.ParameterRange
        N = 12; worst = 0.0; frac = 0; onbed = True
        for i in range(N+1):
            for j in range(N+1):
                uu = u0 + i*(u1-u0)/N; vv = v0 + j*(v1-v0)/N
                p = f.valueAt(uu, vv)
                if p.z - zmin > 0.05: onbed = False
                n = f.normalAt(uu, vv)
                if f.Orientation == "Reversed": n = n.negative()
                c = -n.z
                if c > 0.7071: frac += 1
        if onbed: continue
        share = frac / ((N+1)**2)
        if share > 0:
            rows.append((round(f.Area*share,1), f.Surface.TypeId.split("::")[-1], [round(x,1) for x in (f.CenterOfMass.x, f.CenterOfMass.y, f.CenterOfMass.z)]))
    rows.sort(reverse=True)
    tot = sum(r[0] for r in rows)
    out.write("total overhang (45deg rule, up build) %.1f over %d faces\n" % (tot, len(rows)))
    for r in rows[:20]: out.write("  %s\n" % (r,))
except Exception:
    out.write(traceback.format_exc())
out.close()
