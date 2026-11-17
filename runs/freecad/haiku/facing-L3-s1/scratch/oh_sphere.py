import traceback, math, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/oh_sphere.txt", "w")
def overhang(shape, d, N=10):
    hmin = min(v.Point.dot(d) for v in shape.Vertexes)
    tot = 0.0
    for f in shape.Faces:
        if f.CenterOfMass.dot(d) - hmin < 0.05 and f.Surface.TypeId == "Part::GeomPlane":
            # bed contact only if whole face sits at min height
            if all(abs(v.Point.dot(d) - hmin) < 0.05 for v in f.Vertexes):
                continue
        u0,u1,v0,v1 = f.ParameterRange
        n_s = 0; a = 0.0
        for i in range(N):
            for j in range(N):
                uu = u0 + (i+0.5)*(u1-u0)/N; vv = v0 + (j+0.5)*(v1-v0)/N
                n = f.normalAt(uu, vv)
                if -n.dot(d) > 0.7071 and f.valueAt(uu,vv).dot(d) - hmin > 0.05:
                    n_s += 1
        tot += f.Area * n_s / (N*N)
    return tot
try:
    s = Part.read(D + "output.step").Solids[0]
    best = []
    for th in range(0, 181, 15):
        for ph in range(0, 360, 15):
            t = math.radians(th); p = math.radians(ph)
            d = V(math.sin(t)*math.cos(p), math.sin(t)*math.sin(p), math.cos(t))
            best.append((round(overhang(s, d),1), th, ph))
    best.sort()
    out.write("best 10: %s\n" % best[:10])
    out.write("z-up: %s\n" % [b for b in best if b[1]==0])
    out.write("horizontal best: %s\n" % [b for b in best if b[1]==90][:5])
except Exception:
    out.write(traceback.format_exc())
out.close()
