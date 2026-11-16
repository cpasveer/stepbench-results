from build123d import import_step, Vector
import itertools, math
p = import_step("/home/user/stepbench-runs/tasks/pipe-L1-s1/output.step")
faces = p.faces()
info = []
for f in faces:
    try:
        n = f.normal_at(f.center())
        info.append((f, n, f.center(), f.geom_type, f.area))
    except Exception as e:
        info.append((f, None, f.center(), f.geom_type, f.area))
for (f1,n1,c1,t1,a1),(f2,n2,c2,t2,a2) in itertools.combinations(info,2):
    if n1 is None or n2 is None: continue
    if t1.name!="PLANE" or t2.name!="PLANE": continue
    ang = math.degrees(math.acos(max(-1,min(1,n1.dot(n2)))))
    if 3 < ang < 87:
        # check faces touch: distance between centres small-ish relative to areas
        d = (c1-c2).length
        if d < 25:
            print(f"angle_normals {ang:6.1f}  areas {a1:8.2f} {a2:8.2f}  c1 {(round(c1.X,1),round(c1.Y,1),round(c1.Z,1))} n1 {(round(n1.X,2),round(n1.Y,2),round(n1.Z,2))}  c2 {(round(c2.X,1),round(c2.Y,1),round(c2.Z,1))} n2 {(round(n2.X,2),round(n2.Y,2),round(n2.Z,2))}")
