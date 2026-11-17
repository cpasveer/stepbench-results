import json, math, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); P=T["poses"][0]
OA=P["placements"]["A"]["origin"]; RA=P["placements"]["A"]["rotation"]
s=Part.read("output.step")
print("valid",s.isValid(),"solids",len(s.Solids),"vol",s.Volume,"faces",len(s.Faces),"edges",len(s.Edges))
print("min edge",min(e.Length for e in s.Edges),"min face",min(f.Area for f in s.Faces))
# build dir in world: -x local
d=V(RA[0][1],RA[1][1],0)
bedx=None
print("down-facing faces (angle from build axis of normal <135 => overhang>45):")
for f in s.Faces:
    u0,u1,v0,v1=f.ParameterRange
    n=f.normalAt((u0+u1)/2,(v0+v1)/2)
    c=n.dot(d)
    if c<-0.01:
        ang=math.degrees(math.acos(max(-1,min(1,-c))))  # 0 = flat down
        print("  %-10s area %8.2f  angle-from-down %5.1f  com %s"%(f.Surface.__class__.__name__,f.Area,ang,[round(x,1) for x in f.CenterOfMass]))
for n in "AB":
    fx=Part.read(f"fixed_{n}.step")
    print(n,"common vol",s.common(fx).Volume)
for b in P["resolved"]["bolts"]:
    p=V(*b["position"]); a=V(*b["axis"])
    grip=3.2 if b["id"][0]=="A" else 6.2
    hd=b["head_diameter"]; hh=b["head_height"]
    head=Part.makeCylinder(hd/2+0.5,hh+1,p+a*grip,a)
    tool=Part.makeCylinder(hd/2,30,p+a*(grip+hh),a)
    ring=Part.makeCylinder(hd/2,0.2,p-a*0.1,a).common(s)
    lo=b["clearance_diameter_range"][0]
    full=math.pi*(hd*hd-lo*lo)/4*0.1
    print(b["id"],"head common",round(s.common(head).Volume,4),"tool common",round(s.common(tool).Volume,4),"dist tool",round(s.distToShape(tool)[0],3),"seat frac ~",round(ring.Volume/ (math.pi*(hd*hd-(lo+0.2)**2)/4*0.1),3))

# seat inside
cB=V(*P["placements"]["B"]["origin"]); cA=V(*OA)
top=s.common(Part.makeBox(400,400,0.02,V(-350,-50,-25.52)))
print("B seat area",top.Volume/0.02,"outside r41.25:",top.cut(Part.makeCylinder(41.25,1,cB-V(0,0,0.5),V(0,0,1))).Volume/0.02)
bot=s.common(Part.makeBox(400,400,0.02,V(-350,-50,-94.0)))
sq=Part.makeBox(71,71,1,V(-35.5,-35.5,-0.5)); m=App.Matrix(RA[0][0],RA[0][1],0,OA[0],RA[1][0],RA[1][1],0,OA[1],0,0,1,OA[2]); sq.transformShape(m)
print("A seat area",bot.Volume/0.02,"outside:",bot.cut(sq).Volume/0.02)
# key sweep
import math
for b in P["resolved"]["bolts"]:
    p=V(*b["position"]); a=V(*b["axis"]); grip=3.2 if b["id"][0]=="A" else 6.2
    key,short,long_=(3,20,63) if b["size"]=="M4" else (4,25,70)
    best=0; run=0; free=[]
    for eng in (0.6,):
        zc=grip+b["head_height"]*(1-eng)+short-key/2
        for k in range(72):
            ang=math.radians(k*5); dl=V(RA[0][0]*math.cos(ang)+RA[0][1]*math.sin(ang),RA[1][0]*math.cos(ang)+RA[1][1]*math.sin(ang),0)
            leg=Part.makeCylinder(key/2*1.16,long_,p+a*zc,dl)
            free.append(s.distToShape(leg)[0]>0.2)
    ff=free+free; 
    for v in ff:
        run=run+1 if v else 0; best=max(best,run)
    print(b["id"],"free key sweep deg ~",min(best,72)*5)
