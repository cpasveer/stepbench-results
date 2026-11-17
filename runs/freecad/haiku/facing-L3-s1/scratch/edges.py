import traceback, math, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/edges.txt", "w")
try:
    s = Part.read(D + "output.step").Solids[0]
    # map edge -> faces
    emap = {}
    for f in s.Faces:
        for e in f.Edges:
            emap.setdefault(e.hashCode(), []).append(f)
    rows = []
    for e in s.Edges:
        fs = emap.get(e.hashCode(), [])
        if len(fs) != 2: continue
        m = e.valueAt(e.FirstParameter + 0.5*(e.LastParameter-e.FirstParameter))
        n1 = fs[0].normalAt(*fs[0].Surface.parameter(m)) if False else None
        try:
            p1 = fs[0].Surface.parameter(m); p2 = fs[1].Surface.parameter(m)
            n1 = fs[0].normalAt(*p1); n2 = fs[1].normalAt(*p2)
        except Exception:
            continue
        c = max(-1, min(1, n1.dot(n2)))
        ang = math.degrees(math.acos(c))   # angle between normals (180 = flat)
        dihedral_gap = 180 - ang           # 0 = flat, 90 = right-angle corner
        if e.Length < 0.3: continue
        rows.append((round(dihedral_gap,1), round(e.Length,2), [round(x,1) for x in (m.x,m.y,m.z)]))
    rows.sort(key=lambda r: -r[0])
    out.write("edges with printed value < 75 (small = sharp corner or crevice):\n")
    for r in rows:
        if r[0] < 75: out.write("  %s\n" % (r,))
    out.write("total edges %d\n" % len(rows))
except Exception:
    out.write(traceback.format_exc())
out.close()
