import json, numpy as np
T = json.load(open("task.json"))
P = T["poses"][0]["placements"]
A = P["A"]; B = P["B"]
RA = np.array(A["rotation"]).T  # columns = local axes? check below
RAr = np.array(A["rotation"])
OA = np.array(A["origin"]); OB = np.array(B["origin"])
nB = np.array(T["poses"][0]["resolved"]["mounting_faces"][0]["normal"])
print("pipe local cols: axis", RAr[:,0], "y", RAr[:,1], "z", RAr[:,2])
bolts = T["poses"][0]["resolved"]["bolts"]
def loc(p): return RAr.T @ (np.array(p)-OA)
for b in bolts:
    p = np.array(b["position"]); ax=np.array(b["axis"])
    if b["id"].startswith("A"):
        print(b["id"], "local pos", np.round(loc(p),3), "local axis", np.round(RAr.T@ax,3))
print("B origin in pipe local", np.round(loc(OB),3))
dirB = np.array([0,0,0.0])
# B face frame
RBr = np.array(B["rotation"])
print("B rot cols", RBr[:,0], RBr[:,1], RBr[:,2])
for b in bolts:
    if b["id"].startswith("B"):
        print(b["id"], "B-local", np.round(RBr.T@(np.array(b["position"])-OB),3))
print("pipe axis pos vs B: ", np.round(RAr.T@(OB-OA),3))
