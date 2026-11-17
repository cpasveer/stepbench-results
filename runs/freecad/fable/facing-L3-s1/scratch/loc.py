import json
T=json.load(open("task.json")); P=T["poses"][0]
OA=P["placements"]["A"]["origin"]; RA=P["placements"]["A"]["rotation"]
def tl(p):
    d=[p[i]-OA[i] for i in range(3)]
    return [round(sum(RA[j][i]*d[j] for j in range(3)),3) for i in range(3)]
print("CB",tl(P["placements"]["B"]["origin"]))
for b in P["resolved"]["bolts"]: print(b["id"],tl(b["position"]))
