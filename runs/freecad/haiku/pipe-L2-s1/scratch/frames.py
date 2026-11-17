import math
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1/scratch/geom.py").read().split("for b in P")[0].replace('print("pipe X"','0 and print("pipe X"'))
B0 = pl["B"]["origin"]
n = [0.3885173161164798,0.9214414224885092,0.0]
Bx=[0,0,1.0]; By=[0.9214414224885092,-0.3885173161164798,0]; Bz=n
def toB(p):
    d=sub(p,B0); return [round(dot(d,Bx),2),round(dot(d,By),2),round(dot(d,Bz),2)]
def pw(ax,rad,theta):  # pipe point: axial ax, radial r, angle theta from Z_pipe toward Y_pipe
    dirv = add(sc(math.cos(math.radians(theta)),Z), sc(math.sin(math.radians(theta)),Y))
    return add(add(O,sc(ax,X)), sc(rad,dirv))
for r in [34,42,51,60,70,80]:
    print("crest r=%d" % r, "B-frame", toB(pw(0,r,0)), "world", [round(v,2) for v in pw(0,r,0)])
print("B0 in pipe-local:", [round(dot(sub(B0,O),X),2),round(dot(sub(B0,O),Y),2),round(dot(sub(B0,O),Z),2)])
print("n in pipe-local:", [round(dot(n,X),3),round(dot(n,Y),3),round(dot(n,Z),3)])
print("Bx in pipe-local:", [round(dot(Bx,X),3),round(dot(Bx,Y),3),round(dot(Bx,Z),3)])
print("By in pipe-local:", [round(dot(By,X),3),round(dot(By,Y),3),round(dot(By,Z),3)])
for b in P["bolts"]:
    print(b["id"], "B-frame", toB(b["position"]), "axis-B", [round(dot(b["axis"],v),3) for v in (Bx,By,Bz)])
