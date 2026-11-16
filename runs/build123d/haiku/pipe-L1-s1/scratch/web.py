import json, numpy as np
exec(open("/home/user/stepbench-runs/tasks/pipe-L1-s1/scratch/geom.py").read().split("for h in")[0])
top_B = oB + 6*RB[:,2]
pad_top = oA + RA@np.array([0,0,42.0])
pad_out = RA@np.array([0,0,1.0])
print("B plate top centre", top_B.round(2))
print("pad top centre", pad_top.round(2), "pad outward", pad_out.round(3))
v = top_B - pad_top
print("length", np.linalg.norm(v).round(2), "dir", (v/np.linalg.norm(v)).round(3))
print("angle to pad outward (deg)", np.degrees(np.arccos(np.clip(v@pad_out/np.linalg.norm(v),-1,1))).round(1))
print("angle to B normal (deg)", np.degrees(np.arccos(np.clip(v@RB[:,2]/np.linalg.norm(v),-1,1))).round(1))
print("pipe local of B origin", (RA.T@(oB-oA)).round(2), " B plate top in pipe local", (RA.T@(top_B-oA)).round(2))
print("pad top in B local", (RB.T@(pad_top-oB)).round(2))
