exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan.py").read().split("def build_var")[0])
names=[]
for bid in BOLT: names += [bid+"_head", bid+"_tool"]
def uvn2xyz(u,v,n): return CB+UB*u+ZB*v+NB*n
def build_roots(roots, s_strap=6.0, sec=10.0):
    axis_of={k:V(*BOLT[k]["axis"]) for k in BOLT}
    body=None; rs=[]
    for pair in (("A1","A2"),("A3","A4")):
        r=axis_of[pair[0]]; tg=AX.cross(r); tg.normalize()
        blk=frame_box(P0-AX*23.5-tg*8.0,AX,tg,r,47.0,16.0,39.0).cut(pipe)
        body=blk if body is None else body.fuse(blk); rs.append(r)
    body=body.fuse(frame_box(CB-UB*17-ZB*17,UB,NB,ZB,34.0,4.0,34.0))
    r1,r3=rs
    targets=[(P0+r1*38.0, roots[0]), (P0-AX*s_strap+r3*38.0, roots[1]), (P0+AX*s_strap+r3*38.0, roots[2])]
    for tgt,root in targets:
        body=body.fuse(bar(uvn2xyz(*root), tgt, sec))
    return body.removeSplitter()
import sys
cands = {
 "A": [(0,8,4),(-5,11.7,4),(-4,10,4)],
 "B": [(0,8,4),(-4,9,4),(-3,8,4)],
}
for k,roots in cands.items():
    b=build_roots(roots)
    hits=[(names[i], round(b.common(kk).Volume,2)) for i,kk in enumerate(keep) if b.common(kk).Volume>0.01]
    print("cand",k,roots,"hits",hits,"pipe",round(b.common(pipe).Volume,3), "vol", round(b.Volume,0))
