import os, math
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
ns = {"__file__": os.path.join(H, "submission.py")}
exec(open(os.path.join(H, "submission.py")).read(), ns)   # defines build_in_a etc, rewrites output.step
M_W = ns["M_WORLD_FROM_A"]; M_AB = ns["M_A_FROM_B"]
solid_w = Part.read(os.path.join(H, "output.step"))
print("valid", solid_w.isValid(), "solids", len(solid_w.Solids), "volume", round(solid_w.Volume,1), "faces", len(solid_w.Faces))
M_inv = M_W.inverse()
A_loc = solid_w.transformShape(M_inv)
fixA = Part.read(os.path.join(H,"fixed_A.step")).transformShape(M_inv)
fixB = Part.read(os.path.join(H,"fixed_B.step")).transformShape(M_inv)
print("overlap A", round(A_loc.common(fixA).Volume,4), "overlap B", round(A_loc.common(fixB).Volume,4))

def slab(z0, z1, r=200):
    return Part.makeCylinder(r, z1-z0, V(0,0,z0), V(0,0,1))
# seat check A: section at z in [0,0.01]
sA = A_loc.common(Part.makeCylinder(100,0.01,V(0,0,0),V(0,0,1)))
inside = sA.common(Part.makeCylinder(36.25,0.01,V(0,0,0),V(0,0,1)))
print("A seat area", round(sA.Volume/0.01,1), "inside 36.25 area", round(inside.Volume/0.01,1))
# seat check B: B-frame section: transform solid into B-local
M_BA_inv = M_AB.inverse()
B_loc = A_loc.transformShape(M_BA_inv)
sB = B_loc.common(Part.makeBox(300,300,0.01,V(-150,-150,0)))
inB = sB.common(Part.makeBox(44,94,0.01,V(-22,-47,0)))
print("B seat area", round(sB.Volume/0.01,1), "inside 1mm-inset area", round(inB.Volume/0.01,1))
# ring coverage around bolts (A: hole 2.75 .. head 4.25 ; B: 3.3 .. 5.0)
def ring_cov(sec, x, y, r0, r1):
    ring = Part.makeCylinder(r1,0.01,V(x,y,0),V(0,0,1)).cut(Part.makeCylinder(r0,0.01,V(x,y,0),V(0,0,1)))
    return round(sec.common(ring).Volume / ring.Volume,3)
A_bolts=[(30.0,0.0),(15.0,25.980762114),(-15.0,25.980762114),(-30.0,0.0),(-15.0,-25.980762114),(15.0,-25.980762114)]
print("A ring coverage", [ring_cov(sA,x,y,2.75,4.25) for x,y in A_bolts])
sBb = B_loc.common(Part.makeBox(300,300,0.01,V(-150,-150,0)))
print("B ring coverage", [ring_cov(sBb,x,y,3.3,5.0) for x,y in [(-15,-40),(15,-40),(-15,40),(15,40)]])

# bolt list in A-local: (pos, axis, head_top_height, head_r, leg_centre_height, leg_half, key_len)
bolts=[]
for x,y in A_bolts:
    bolts.append(("A", V(x,y,0), V(0,0,1), 8+5.0, 2.75, 4.25, 8+5+20.0, 2.0, 70.0))
for x,y in [(-15,-40),(15,-40),(-15,40),(15,40)]:
    p = M_AB.multVec(V(x,y,0)); d = M_AB.multVec(V(0,0,1)) - M_AB.multVec(V(0,0,0))
    bolts.append(("B", p, d, 8+6.0, 3.3, 5.0, 8+6+22.0, 2.5, 80.0))

def perp(d):
    d = V(d); d.normalize()
    t = V(1,0,0) if abs(d.x) < 0.9 else V(0,1,0)
    e1 = d.cross(t); e1.normalize(); e2 = d.cross(e1); e2.normalize()
    return d, e1, e2

obst_all = A_loc.fuse(fixA).fuse(fixB)
for name, p, d, htop, r_hole, r_head, hleg, rleg, L in bolts:
    dn, e1, e2 = perp(d)
    # head room: cylinder r_head+0.5 from htop-H .. htop+1 above the bearing
    H_head = 5.0 if name=="A" else 6.0
    # bearing plane position: pad top at distance (htop - H_head) along axis from the seat
    base = p + dn*((htop-H_head)+0.01)
    room = Part.makeCylinder(r_head+0.5, H_head+1.0-0.01, base, dn)
    room_ov = round(A_loc.common(room).Volume,4); c_=A_loc.common(room); print("DBG", name, c_.BoundBox, [round(s.Volume,2) for s in c_.Solids], "A_loc solids", len(A_loc.Solids), A_loc.BoundBox)
    # key sweep: 0..360 in 3 deg steps, leg at height hleg above seat
    blocked=[]
    for k in range(0,360,3):
        th=math.radians(k)
        rad = e1*math.cos(th) + e2*math.sin(th)
        base_leg = p + dn*hleg
        leg = Part.makeCylinder(rleg, L, base_leg, rad)
        if leg.common(obst_all).Volume > 0.01:
            blocked.append(k)
    # longest free arc
    free=[k for k in range(0,360,3) if k not in blocked]
    best=0; cur=0; 
    seq = free+free
    run=0; best=0
    for i in range(len(seq)):
        if i>0 and seq[i]==seq[i-1]+3: run+=1
        else: run=1
        best=max(best,run)
    print(name, "head_room_overlap", room_ov, "blocked deg", len(blocked)*3, "longest free arc deg", best*3 if best else 0)
