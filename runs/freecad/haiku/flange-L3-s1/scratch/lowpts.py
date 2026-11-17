import FreeCAD as App, Part, math
V=App.Vector
s=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/output.step")
nA=V(0.7384553406,0.6743023876,0); Z=V(0,0,1)
u=V(0.5,0.5,-0.7071067812); u=u*(1/u.Length)
hs=sorted([(v.Point.dot(u), v.Point) for v in s.Vertexes], key=lambda t:t[0])
print("hmin", round(hs[0][0],2), "hmax", round(hs[-1][0],2))
for h,p in hs[:8]:
    print("LOW h=%.2f p=(%.2f,%.2f,%.2f) a=%.2f" % (h,p.x,p.y,p.z,p.dot(nA)))
