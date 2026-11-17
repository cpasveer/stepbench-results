import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
cands = {"n_B": V(0.3885,0.9214,0), "-n_B": V(-0.3885,-0.9214,0), "pipeX": V(0.2488,0.5901,0.7681), "-pipeX": V(-0.2488,-0.5901,-0.7681), "z": V(0,0,1), "-z": V(0,0,-1)}
for name,b in cands.items():
    b.normalize()
    tot=0; rows=[]
    for f in s.Faces:
        u0,u1,v0,v1=f.ParameterRange
        for i in range(3):
            for j in range(3):
                n = f.normalAt(u0+(u1-u0)*(i+0.5)/3, v0+(v1-v0)*(j+0.5)/3)
                if n.dot(b) < -0.7071:
                    tot += f.Area/9
                    rows.append((round(f.Area/9,1), f.Surface.__class__.__name__))
    print("build", name, "overhang mm2", round(tot,1), "faces", len(rows))
