import json,sys
r=json.load(open("attempts/%s/result.json"%sys.argv[1]))
for c in r["checks"]:
    if c["name"]=="STIFFNESS": print(json.dumps(c["details"]["stages"]), c["reason"][:60])
    if c["name"] in ("FDM_GAP_MIN","FDM_WALL_MIN","Q_SHALLOW_JOINT","Q_CREASE","Q_THIN_WEDGE","Q_STUB","FDM_OVERHANG","Q_SMALL_STEP"): print(c["name"],c["reason"])
print(r["info"].get("material_used"))
