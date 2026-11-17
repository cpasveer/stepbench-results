import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
from frame import *
E=2500.0; G=E/2.6
r1=A["A1"]; r3=A["A3"]
def S(r, s, rad=36): return P0 + s*AX + rad*r
def uvn(p):
    d=p-CB; return np.array([d@UB, d@ZB, d@NB])
hubxy = (-4.0,-4.0)
plate_pt = CB + UB*hubxy[0] + ZB*hubxy[1] + NB*4.0
for hh in [24.0, 30.0, 36.0]:
    hub_top = CB + UB*hubxy[0] + ZB*hubxy[1] + NB*hh
    legs = [S(r1,0), S(r3,-6), S(r3,6)]
    pts=[plate_pt, hub_top]+legs
    mem=[(0,1,100,833,141)]+[(1,2+i,64,64**0*0+ (8**4)/12, 0.141*8**4) for i in range(3)]
    mem=[(0,1,100,833,141)]+[(1,2+i,64,8**4/12,0.141*8**4) for i in range(3)]
    res=solve(pts, mem, fixed={2,3,4}, load_node=0, E=E, G=G)
    k=[round(1/v,1) for v in res.values()]
    # joint angles: leg angle from build NB at hub
    angs=[]
    for L in legs:
        d=(L-hub_top)/np.linalg.norm(L-hub_top); angs.append(round(float(np.degrees(np.arccos(d@NB))),1))
    print("hub n=%.0f" % hh, "stiff (x,y,z) N/mm", k, "min", round(min(k),1), "leg angles from NB", angs, "uvn leg ends", [np.round(uvn(l),1).tolist() for l in legs])
print("plate_pt uvn", uvn(plate_pt))
