import sys, json, itertools
sys.argv=["x"]
SRC=open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/submission.py").read()
SRC=SRC[:SRC.index("OUT = os.path.join(HERE")]
g={"__file__":"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/submission.py"}
exec(SRC, g)
V=g["V"]; Part=g["Part"]; App=g["App"]
BOLT=g["BOLT"]; CB=g["CB"]; UB=g["UB"]; ZB=g["ZB"]; NB=g["NB"]; P0=g["P0"]; AX=g["AX"]
frame_box=g["frame_box"]; bar=g["bar"]
pipe=Part.makeCylinder(34.0,300,P0-AX*150,AX)
keep=[]
for bid,b in BOLT.items():
    pos=V(*b["position"]); ax=V(*b["axis"]); size=b["size"]
    hd,hh=(8.5,5.0) if size=="M5" else (7.0,4.0)
    seat = 5.0 if bid[0]=="A" else 4.0
    seatp = pos+ax*seat
    keep.append(Part.makeCylinder(hd/2+0.5, hh+1.0, seatp, ax))
    keep.append(Part.makeCylinder(hd/2, 30.0, seatp+ax*hh, ax))
keepc=Part.makeCompound(keep)
def build_var(hub_u, hub_v, hub_top_h, s_strap=6.0):
    axis_of={k:V(*BOLT[k]["axis"]) for k in BOLT}
    body=None; strap_pts=[]
    for pair in (("A1","A2"),("A3","A4")):
        r=axis_of[pair[0]]; tg=AX.cross(r); tg.normalize()
        origin=P0-AX*23.5-tg*8.0
        blk=frame_box(origin,AX,tg,r,47.0,16.0,39.0)
        strap=blk.cut(pipe)
        body=strap if body is None else body.fuse(strap)
        strap_pts.append(r)
    plate=frame_box(CB-UB*17-ZB*17,UB,NB,ZB,34.0,4.0,34.0)
    body=body.fuse(plate)
    hb=CB+UB*hub_u+ZB*hub_v+NB*2.0
    ht=CB+UB*hub_u+ZB*hub_v+NB*hub_top_h
    body=body.fuse(bar(hb,ht,10.0))
    r1,r3=strap_pts
    targets=[P0+r1*38.0, P0+AX*(-s_strap)+r3*38.0, P0+AX*s_strap+r3*38.0]
    for t in targets: body=body.fuse(bar(ht,t,10.0))
    return body
res=[]
for hu in [-6,-4,-2,0,2,4,6]:
  for hv in [-12,-10,-8,-6,-4,-2,0,2,4]:
    for ht in [28.0, 32.0]:
        b=build_var(hu,hv,ht).removeSplitter()
        ov=b.common(keepc).Volume
        pv=b.common(pipe).Volume
        ok = ov<0.5 and pv<0.01
        res.append((round(ov,1), round(pv,2), hu, hv, ht, ok))
good=[r for r in res if r[5]]
print("total", len(res), "clear", len(good))
for r in good[:40]: print(r)
