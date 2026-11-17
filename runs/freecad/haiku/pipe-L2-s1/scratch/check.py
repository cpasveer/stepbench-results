import json, math, os, sys
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
T = json.load(open(TD + "/task.json"))
POSE = T["poses"][0]; BOLTS = POSE["resolved"]["bolts"]
body = Part.read(TD + "/output.step").Solids[0]
fa = Part.read(TD + "/fixed_A.step"); fb = Part.read(TD + "/fixed_B.step")
print("valid", body.isValid(), "vol", round(body.Volume,1), "bbox", body.BoundBox)
print("overlap A mm3", round(body.common(fa).Volume,4), " overlap B mm3", round(body.common(fb).Volume,4))
PL = POSE["placements"]
OA = V(*PL["A"]["origin"]); RA = PL["A"]["rotation"]
XA = V(RA[0][0],RA[1][0],RA[2][0]); YA = V(RA[0][1],RA[1][1],RA[2][1]); ZA = V(RA[0][2],RA[1][2],RA[2][2])
B0 = V(*PL["B"]["origin"]); nB = V(*BOLTS[4]["axis"])
# seating: ring points between hole and head radius, offset 0.2 mm into the part's side
def ring_pts(c, a, r0, r1, n=24, m=4):
    e1 = V(0,0,1).cross(a)
    if e1.Length < 1e-6: e1 = V(1,0,0).cross(a)
    e1.normalize(); e2 = a.cross(e1)
    pts=[]
    for i in range(n):
        for j in range(m):
            th = 2*math.pi*i/n; rr = r0 + (r1-r0)*(j+0.5)/m
            pts.append(c + e1*(rr*math.cos(th)) + e2*(rr*math.sin(th)))
    return pts
def cover(bolt, surf_normal_fn, hr_ring=(None,None)):
    c=V(*bolt["position"]); a=V(*bolt["axis"])
    hd = bolt["head_diameter"]/2
    r0 = (bolt["clearance_diameter_range"][0]+bolt["clearance_diameter_range"][1])/4
    pts = ring_pts(c, a, r0, hd)
    inside=0
    for p in pts:
        # probe a point on the fixed surface, step 0.2 mm toward part (along -normal of fixed surf=+axis)
        q = surf_normal_fn(p)
        if body.isInside(q, 1e-6, True): inside+=1
    return inside/len(pts)
# A: fixed surface is the pipe; part sits outward from pipe so probe inward? use axis: part is at +axis side of surface
for b in BOLTS:
    if b["size"]=="M5":
        c=V(*b["position"]); a=V(*b["axis"])
        def probe(p):
            # point slightly outside the pipe surface (radial from axis)
            d = p - OA; ax = d.dot(XA); perp = d - XA*ax
            rad = perp.normalize() if perp.Length>0 else a
            return p + rad*0.2
        print(b["id"], "seat coverage (pipe)", round(cover(b, probe),3))
    else:
        def probe(p):
            return p + nB*0.2
        print(b["id"], "seat coverage (B)", round(cover(b, probe),3))
