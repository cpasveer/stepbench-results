import itertools
src=open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/submission.py").read()
src=src[:src.index("OUT = os.path.join(HERE")]
g={"__file__":"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/submission.py"}
exec(src,g)
V=g["V"]; Part=g["Part"]; BOLT=g["BOLT"]; CB=g["CB"]; UB=g["UB"]; ZB=g["ZB"]; NB=g["NB"]; P0=g["P0"]; AX=g["AX"]
bar=g["bar"]; frame_box=g["frame_box"]; pipe=Part.makeCylinder(34.0,300,P0-AX*150,AX)
keep=[]
for bid,b in BOLT.items():
    pos=V(*b["position"]); ax=V(*b["axis"]); hd,hh=(8.5,5.0) if b["size"]=="M5" else (7.0,4.0)
    seat=5.0 if bid[0]=="A" else 4.0; sp=pos+ax*seat
    keep.append(Part.makeCylinder(hd/2+0.5,hh+1.0,sp,ax)); keep.append(Part.makeCylinder(hd/2,30.0,sp+ax*hh,ax))
keepc=Part.makeCompound(keep)
r1=V(*BOLT["A1"]["axis"]); r3=V(*BOLT["A3"]["axis"])
def leg_ok(root_uv, tgt, sec=11.0):
    root=CB+UB*root_uv[0]+ZB*root_uv[1]+NB*4.0
    L=bar(root,tgt,sec)
    return round(L.common(keepc).Volume,2)
print("A:")
for ru,rv in [(1,8),(2,8),(0,6),(-2,8),(2,6),(4,8)]:
    print("  root",(ru,rv), leg_ok((ru,rv), P0+r1*38.5))
print("B (strap3 s=-s):")
for ru,rv in [(3.5,-16),(2,-16),(0,-16),(4,-15),(6,-16),(-2,-16),(6,-14)]:
    for s in [-6,-8,-10]:
        print("  root",(ru,rv),"s",s, leg_ok((ru,rv), P0-AX*s*(-1)*(-1)+r3*38.5 if False else P0+AX*s+r3*38.5))
print("C (strap3 s=+):")
for ru,rv in [(-6,8),(-8,8),(-4,10),(-6,10),(-8,6),(-10,8),(0,10)]:
    for s in [6,8,10]:
        print("  root",(ru,rv),"s",s, leg_ok((ru,rv), P0+AX*s+r3*38.5))
