import json, math, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
T = json.load(open(D + "task.json"))["poses"][0]["resolved"]
A = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "A"]
B = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "B"]
ca = (-126.0, 127.5); cb = (-124.18164228311065, 135.11206773573574)
P = ((ca[0]+cb[0])/2, (ca[1]+cb[1])/2)
def strip(p0, p1, w, z0, t):
    dx, dy = p1[0]-p0[0], p1[1]-p0[1]; L = math.hypot(dx, dy)
    nx, ny = -dy/L*w/2, dx/L*w/2
    pts = [V(p0[0]+nx, p0[1]+ny, z0), V(p1[0]+nx, p1[1]+ny, z0), V(p1[0]-nx, p1[1]-ny, z0), V(p0[0]-nx, p0[1]-ny, z0), V(p0[0]+nx, p0[1]+ny, z0)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, t))
parts = [Part.makeCylinder(8, -25.5+94, V(P[0], P[1], -94), V(0,0,1))]
for q in A:
    parts.append(Part.makeCylinder(5, 6, V(q[0], q[1], -94), V(0,0,1)))
    parts.append(strip(P, q, 6, -94, 6))
for q in B:
    parts.append(Part.makeCylinder(6, 6, V(q[0], q[1], -31.5), V(0,0,1)))
    parts.append(strip(P, q, 8, -31.5, 6))
s = parts[0]
for p in parts[1:]:
    s = s.fuse(p)
s = s.removeSplitter()
s.exportStep(D + "scratch/fe/v1.step")
print("v1 volume", s.Volume, "valid", s.isValid(), "solids", len(s.Solids))
