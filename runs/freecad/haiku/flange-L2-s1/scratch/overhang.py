import math, sys, os
import FreeCAD as App, Part
HERE = "/home/user/stepbench-runs/tasks-freecad/flange-L2-s1"
ns = {"__name__": "design", "__file__": HERE + "/submission.py"}
exec(open(HERE + "/submission.py").read(), ns)
body = ns["build"]().removeSplitter()
V = App.Vector
# precompute per-face samples: (point, outward normal, weight)
samples = []
for f in body.Faces:
    u0, u1, v0, v1 = f.ParameterRange
    m = 6
    pts = []
    for i in range(m):
        for j in range(m):
            uu = u0 + (u1-u0)*(i+0.5)/m; vv = v0 + (v1-v0)*(j+0.5)/m
            p = f.valueAt(uu, vv); n = f.normalAt(uu, vv)
            if body.isInside(p + n*0.05, 1e-6, True):
                n = n * -1
            pts.append((p, n))
    w = f.Area / len(pts)
    fz = [v.Point for v in f.Vertexes]
    samples.append((pts, w, fz))
allpts = [v.Point for v in body.Vertexes]
def overhang(u, tol_bed=0.3, ang=45.0):
    u = u.normalize()
    zmin = min(p.dot(u) for p in allpts)
    area = 0.0
    for pts, w, fz in samples:
        if max(p.dot(u) for p in fz) - zmin < tol_bed:
            continue
        for p, n in pts:
            if -n.dot(u) > math.cos(math.radians(ang)):
                area += w
    return area
if __name__ == "__main__" or True:
    print("OVH Z-up %.1f" % overhang(V(0,0,1)))
    print("OVH nA-up %.1f" % overhang(V(0.738455,0.674302,0)))
    best = []
    N = 1200
    for i in range(N):
        z = 1 - 2*(i+0.5)/N
        r = math.sqrt(max(0, 1-z*z)); t = math.pi*(1+5**0.5)*i
        u = V(r*math.cos(t), r*math.sin(t), z)
        best.append((overhang(u), u))
    best.sort(key=lambda x: x[0])
    for a, u in best[:10]:
        print("OVH %.1f  u=(%.3f, %.3f, %.3f)" % (a, u.x, u.y, u.z))
    print("TOTAL_AREA %.1f" % body.Area)
