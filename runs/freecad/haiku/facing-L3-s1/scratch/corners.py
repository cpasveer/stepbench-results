import traceback, math, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/corners.txt", "w")
try:
    s = Part.read(D + "output.step").Solids[0]
    rows = []
    for f in s.Faces:
        for w in f.Wires:
            edges = w.OrderedEdges
            n = len(edges)
            for i in range(n):
                e1 = edges[i-1]; e2 = edges[i]
                v = e2.Vertexes[0].Point if e2.Vertexes[0].Point.distanceToPoint(e1.Vertexes[-1].Point) < 1e-4 else e2.Vertexes[-1].Point
                p_prev = e1.valueAt(e1.FirstParameter) if e1.Vertexes[-1].Point.distanceToPoint(v) < 1e-4 else e1.valueAt(e1.LastParameter)
                p_prev = e1.Vertexes[0].Point if e1.Vertexes[-1].Point.distanceToPoint(v) < 1e-4 else e1.Vertexes[-1].Point
                p_next = e2.Vertexes[-1].Point if e2.Vertexes[0].Point.distanceToPoint(v) < 1e-4 else e2.Vertexes[0].Point
                a = p_prev.sub(v); b = p_next.sub(v)
                if a.Length < 1e-6 or b.Length < 1e-6: continue
                ang = math.degrees(math.acos(max(-1, min(1, a.normalize().dot(b.normalize())))))
                if ang < 8:
                    rows.append((round(ang, 2), [round(x, 2) for x in (v.x, v.y, v.z)], round(f.Area, 1), f.Surface.TypeId.split("::")[-1]))
    rows.sort()
    out.write("corners with interior angle < 8 deg: %d\n" % len(rows))
    for r in rows[:20]: out.write("  %s\n" % (r,))
except Exception:
    out.write(traceback.format_exc())
out.close()
