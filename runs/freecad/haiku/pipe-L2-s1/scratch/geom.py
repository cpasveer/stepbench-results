import json, math
T = json.load(open("/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1/task.json"))
P = T["poses"][0]["resolved"]
pl = T["poses"][0]["placements"]
O = pl["A"]["origin"]; R = pl["A"]["rotation"]
X = [R[i][0] for i in range(3)]; Y = [R[i][1] for i in range(3)]; Z=[R[i][2] for i in range(3)]
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def sub(a,b): return [x-y for x,y in zip(a,b)]
def add(a,b): return [x+y for x,y in zip(a,b)]
def sc(s,a): return [s*x for x in a]
def norm(a): l=math.sqrt(dot(a,a)); return [x/l for x in a]
print("pipe X", X, "Y", Y, "Z", Z)
for b in P["bolts"]:
    p=b["position"]; a=b["axis"]
    v=sub(p,O); ax=dot(v,X); perp=sub(v,sc(ax,X))
    r=math.sqrt(dot(perp,perp))
    u=norm(perp)
    print(b["id"], "axial %.2f radial %.2f" % (ax, r), "u_local", [round(dot(u,X),3),round(dot(u,Y),3),round(dot(u,Z),3)],
          "axis.u %.4f" % dot(a,u), "axis_loc", [round(dot(a,X),3),round(dot(a,Y),3),round(dot(a,Z),3)],
          "head", b["head_diameter"], b["head_height"], "grip", b["grip_range"])
