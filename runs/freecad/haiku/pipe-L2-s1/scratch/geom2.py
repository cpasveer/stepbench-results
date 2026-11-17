import json, math
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1/scratch/geom.py").read().split("for b in P")[0].replace('print("pipe X"','0 and print("pipe X"'))
n = [0.3885173161164798,0.9214414224885092,0.0]
B0 = pl["B"]["origin"]
# ray from B bolt centres along their axis; distance to pipe cylinder (radius 34, axis O + t X)
for b in P["bolts"]:
    c=b["position"]; a=b["axis"]
    # solve |perp(c + s a - O)| = 34 for s
    w = sub(c,O); 
    wa = dot(w,X); aa = dot(a,X)
    w_perp = sub(w, sc(wa,X)); a_perp = sub(a, sc(aa,X))
    A = dot(a_perp,a_perp); Bq = 2*dot(w_perp,a_perp); C = dot(w_perp,w_perp)-34*34
    disc = Bq*Bq-4*A*C
    if disc<0: print(b["id"],"no hit"); continue
    s1=(-Bq-math.sqrt(disc))/(2*A); s2=(-Bq+math.sqrt(disc))/(2*A)
    # axial coords at hits
    ax1 = wa + s1*aa; ax2 = wa+s2*aa
    print(b["id"], "ray hits pipe at s=%.1f,%.1f axial=%.1f,%.1f" % (s1,s2,ax1,ax2))
# B face in-plane coords of pipe points: sample pipe surface points, express in B frame
Bx=[0,0,1]; By=[0.9214414224885092,-0.3885173161164798,0]; Bz=n
mins=[]
for i in range(0,360,5):
  for t in range(-130,131,2):
    th=math.radians(i)
    rad=sc(math.cos(th),Y)
    rad=add(rad, sc(math.sin(th),Z))
    p=add(add(O,sc(t,X)), sc(34,rad))
    d=sub(p,B0)
    h=[dot(d,Bx),dot(d,By),dot(d,Bz)]
    if abs(h[0])<=40 and abs(h[1])<=40 and h[2]>-1:
        mins.append((round(h[2],1), round(h[0],1), round(h[1],1), t, i))
mins.sort()
print("pipe points over B footprint (n-height, x, y, axial, angle) smallest:", mins[:8])
print("B local frame check: pipe axis point nearest B", [round(v,2) for v in [dot(sub(O,B0),Bx),dot(sub(O,B0),By),dot(sub(O,B0),Bz)]])
