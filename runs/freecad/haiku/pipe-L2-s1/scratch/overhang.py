import math
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
# candidate build directions: fibonacci sphere
N = 2000
best = []
faces = []
for f in s.Faces:
    # planar faces give exact normals; sample a point for curved faces
    u0,u1,v0,v1 = f.ParameterRange
    K=5
    for i in range(K):
        for j in range(K):
            uu=u0+(u1-u0)*(i+0.5)/K; vv=v0+(v1-v0)*(j+0.5)/K
            n = f.normalAt(uu, vv)
            faces.append((f.Area/(K*K), n, f.Surface.__class__.__name__))
print("faces", len(faces), "types", sorted(set(t for _,_,t in faces)))
res=[]
for i in range(N):
    z = 1 - 2*(i+0.5)/N
    r = math.sqrt(max(0,1-z*z)); phi = math.pi*(1+5**0.5)*i
    b = V(r*math.cos(phi), r*math.sin(phi), z)
    area = 0.0
    for A,n,_ in faces:
        if n.dot(b) < -0.7071:  # normal within 45 deg of straight down
            area += A
    res.append((area,(round(b.x,3),round(b.y,3),round(b.z,3))))
res.sort()
print("min overhang area (mm2) over build directions:", res[:5])
