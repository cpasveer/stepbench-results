import json, math
T = json.load(open("task.json"))
R = T["poses"][0]["resolved"]
P0 = [21.0, 136.5, -7.0]
a = [0.2487956429525063, 0.5900653629615055, 0.7680648380709507]
CB = T["poses"][0]["placements"]["B"]["origin"]
nB = [0.3885173161164798, 0.9214414224885092, 0.0]
u = [0.9214414224885092, -0.3885173161164798, 0.0]
v = [0.0, 0.0, 1.0]
def sub(x,y): return [x[i]-y[i] for i in range(3)]
def dot(x,y): return sum(x[i]*y[i] for i in range(3))
def add(x,y): return [x[i]+y[i] for i in range(3)]
def sc(s,x): return [s*c for c in x]
def loc(p): d=sub(p,CB); return (round(dot(d,u),2), round(dot(d,v),2), round(dot(d,nB),2))
for b in R["bolts"]:
    p=b["position"]; ax=b["axis"]
    d=sub(p,P0)
    s=dot(d,a)
    rad=sub(d,sc(s,a))
    print(b["id"], "axial s=%.2f"%s, "radial=%.2f"%math.sqrt(dot(rad,rad)), "loc(u,v,n)=",loc(p), "axis·nB=%.3f"%dot(ax,nB), "axis·a=%.3f"%dot(ax,a), "axis·radial=%.3f"%dot(ax,sc(1/math.sqrt(dot(rad,rad)),rad)))
for name,ang_vec in [("r1",[0.7093118905965579,-0.6509870200868549,0.27035632165831364]),("r3",[-0.3771547728198181,-0.6713858946506742,0.6379617989560074])]:
    for r in [34,40,44]:
        for s in [-17.5,0,17.5]:
            q=add(add(P0,sc(s,a)),sc(r,ang_vec))
            print(name,"r=",r,"s=",s,"loc=",loc(q))
print("pipe axis point at B plane distance: P0 loc", loc(P0), "axis in n", dot(a,nB))
print("B holes local", [ loc(b["position"])[:2] for b in R["bolts"][4:]])
