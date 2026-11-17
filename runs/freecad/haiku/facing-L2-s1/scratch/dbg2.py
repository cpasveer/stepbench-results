import sys, json, math
sys.argv=["x"]
import FreeCAD as App, Part
src = open("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/submission.py").read()
src = src.split("OUT = os.path.join")[0]
g = {"__file__": "/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/submission.py"}
exec(compile(src, "sub", "exec"), g)
# instrument: rebuild step by step
V = App.Vector
FACES=g["FACES"]; BOLTS=g["BOLTS"]; PLACE=g["PLACE"]
A=FACES[0]["origin"]; B=FACES[1]["origin"]
zA,zB=A[2],B[2]
thetaA = math.degrees(math.atan2(PLACE["A"]["rotation"][1][0], PLACE["A"]["rotation"][0][0]))
plate = Part.makeBox(70,70,8,V(-35,-35,0)); plate.rotate(V(0,0,0),V(0,0,1),thetaA); plate.translate(V(A[0],A[1],zA))
print("plate", plate.isValid())
body = Part.makeCylinder(28.0, zB-(zA+8.0)+1.0, V(B[0],B[1],zA+8.0-1.0), V(0,0,1))
sh = plate.fuse(body); print("plate+body", sh.isValid())
for pp in [BOLTS[k]["position"] for k in ("B1","B2","B3")]:
    pass
print("run full build valid:", g["build"]().isValid())
b = g["build"]()
print("build", b.isValid(), len(b.Solids))
r = b.removeSplitter(); print("after removeSplitter", r.isValid(), len(r.Solids))
r2 = r.Solids[0].removeSplitter(); print("solid removeSplitter", r2.isValid())
r2.exportStep("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/scratch/t.step")
t = Part.read("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/scratch/t.step")
print("roundtrip", t.isValid(), len(t.Solids))
print("fix", r2.isValid(), r2.Volume)
