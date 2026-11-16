import json, numpy as np
T = json.load(open("/home/user/stepbench-runs/tasks/pipe-L1-s1/task.json"))
P = T["poses"][0]["placements"]
A = P["A"]; B = P["B"]
RA = np.array(A["rotation"]); oA = np.array(A["origin"])
RB = np.array(B["rotation"]); oB = np.array(B["origin"])
print("pipe axis (local X in world):", RA[:,0])
print("B normal (local Z in world):", RB[:,2])
for h in T["fixed_parts"][0]["interface"]["hole_frames_local"]:
    p = RA@np.array(h["point"])+oA; a = RA@np.array(h["axis"])
    print("A hole world", p.round(2), "axis", a.round(3))
for b in T["poses"][0]["resolved"]["bolts"]:
    p=np.array(b["position"]); a=np.array(b["axis"])
    d=RA[:,0]; c=oA
    v=p-c; rad=v-(v@d)*d
    print(b["id"], "radial dist", round(np.linalg.norm(rad),3), "radial dir·axis", round((rad/np.linalg.norm(rad))@a,3), "axis·d", round(a@d,3), "local", (RA.T@(p-oA)).round(2))
    print("   B local pos", (RB.T@(p-oB)).round(2), "axis local", (RB.T@a).round(3))
