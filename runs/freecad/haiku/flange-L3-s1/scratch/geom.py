import json, math
T = json.load(open("../task.json"))
R = T["poses"][0]["resolved"]
OA = R["mounting_faces"][0]["origin"]; nA = R["mounting_faces"][0]["normal"]
OB = R["mounting_faces"][1]["origin"]; nB = R["mounting_faces"][1]["normal"]
tA = (-nA[1], nA[0], 0.0)           # horizontal in-plane for A
tB = (nB[1], -nB[0], 0.0)            # horizontal in-plane for B (sign chosen; check)
def d(p,o): return [p[i]-o[i] for i in range(3)]
def dot(a,b): return sum(a[i]*b[i] for i in range(3))
print("gap centre distance", math.sqrt(sum(x*x for x in d(OB,OA))))
print("A plane dist of OB:", dot(d(OB,OA),nA), " B plane dist of OA:", dot(d(OA,OB),nB))
for b in R["bolts"]:
    p = b["position"]; n = b["axis"]
    o = OA if b["id"][0]=="A" else OB
    t = tA if b["id"][0]=="A" else tB
    v = d(p,o)
    print(b["id"], "a=%.2f  t=%.2f  z=%.2f" % (dot(v,n), dot(v,t), v[2]), "head", b["head_diameter"], b["head_height"], "grip", b["grip_range"])
print("nA.nB", dot(nA,nB), "tA", tA, "tB", tB)
print("--- in A frame (a along nA, b along tA, z) ---")
for b in R["bolts"]:
    v = d(b["position"], OA)
    print(b["id"], "a=%.2f b=%.2f z=%.2f" % (dot(v,nA), dot(v,tA), v[2]))
print("--- B faces corners in A frame ---")
for sx in (-23,23):
  for sz in (-48,48):
    p=[OB[0]+sx*tB[0], OB[1]+sx*tB[1], OB[2]+sz]
    v=d(p,OA); print("B corner", sx, sz, "a=%.2f b=%.2f z=%.2f"%(dot(v,nA),dot(v,tA),v[2]))
