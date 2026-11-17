import FreeCAD as App, Part, math
exec(open("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/scratch/overhang.py").read().split("if True:")[0])
res=[]
for i in range(0,181,3):
    for j in range(0,360,3):
        th=math.radians(i); ph=math.radians(j)
        u=V(math.sin(th)*math.cos(ph), math.sin(th)*math.sin(ph), math.cos(th))
        res.append((overhang(u),i,j))
res.sort()
for r in res[:12]:
    th=math.radians(r[1]); ph=math.radians(r[2])
    u=(round(math.sin(th)*math.cos(ph),3), round(math.sin(th)*math.sin(ph),3), round(math.cos(th),3))
    print("OH", round(r[0],1), "theta",r[1],"phi",r[2], "u",u)
