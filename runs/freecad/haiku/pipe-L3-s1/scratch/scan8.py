import itertools
src=open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/submission.py").read()
src=src[:src.index("OUT = os.path.join(HERE")]
g={"__file__":"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/submission.py"}
exec(src,g)
V=g["V"]; Part=g["Part"]; BOLT=g["BOLT"]; CB=g["CB"]; UB=g["UB"]; ZB=g["ZB"]; NB=g["NB"]; P0=g["P0"]; AX=g["AX"]
frame_box=g["frame_box"]; bar=g["bar"]; pipe=Part.makeCylinder(34.0,300,P0-AX*150,AX)
keep=[]
for bid,b in BOLT.items():
    pos=V(*b["position"]); ax=V(*b["axis"]); hd,hh=(8.5,5.0) if b["size"]=="M5" else (7.0,4.0)
    seat=5.0 if bid[0]=="A" else 4.0; sp=pos+ax*seat
    keep.append(Part.makeCylinder(hd/2+0.5,hh+1.0,sp,ax)); keep.append(Part.makeCylinder(hd/2,30.0,sp+ax*hh,ax))
keepc=Part.makeCompound(keep)
axis_of={k:V(*BOLT[k]["axis"]) for k in BOLT}
def base_body():
    body=None; rs=[]
    for pair in (("A1","A2"),("A3","A4")):
        r=axis_of[pair[0]]; tg=AX.cross(r); tg.normalize()
        blk=frame_box(P0-AX*23.5-tg*8.0,AX,tg,r,47.0,16.0,39.0).cut(pipe)
        body=blk if body is None else body.fuse(blk); rs.append(r)
    body=body.fuse(frame_box(CB-UB*17-ZB*17,UB,NB,ZB,34.0,4.0,34.0))
    return body, rs
def legs(ra,rb,rs):
    r1,r3=rs
    ro_a=CB+UB*ra[0]+ZB*ra[1]+NB*2.0; ro_b=CB+UB*rb[0]+ZB*rb[1]+NB*2.0
    return [bar(ro_a,P0-AX*6+r1*37.0,10.0), bar(ro_a,P0+AX*6+r1*37.0,10.0), bar(ro_b,P0+r3*37.0,10.0)]
for ra,rb in [((1,8),(3,-16)),((1,8),(4,-16)),((1,8),(2,-17)),((0,8),(2,-16)),((2,8),(2,-16))]:
    body,rs=base_body()
    for L in legs(ra,rb,rs): body=body.fuse(L)
    body=body.removeSplitter()
    names=[]
    for bid in BOLT: names+=[bid+"_head",bid+"_tool"]
    hits=[(names[i],round(body.common(k).Volume,2)) for i,k in enumerate(keep) if body.common(k).Volume>0.01]
    print("cand",ra,rb,hits,"pipe",round(body.common(pipe).Volume,3))
