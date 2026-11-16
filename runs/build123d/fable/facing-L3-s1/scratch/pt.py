import sys,json; sys.path.insert(0,".")
import submission as S
for p in [(-132.264, 105.517),(-96.285, 124.813),(-95.807, 107.806),(-111.796, 119.268)]:
    print(p,S.to_design(p))
for n in ("0001","0002"):
    r=json.load(open(f"attempts/{n}/result.json"))
    for c in r["checks"]:
        if c["name"] in("FDM_OVERHANG","FDM_GAP_MIN","FDM_WALL_MIN","STIFFNESS","FDM_BED_CONTACT"): print(n,c["name"],c["reason"],json.dumps(c["details"])[:700])
    print([k for k in r.keys()])
    print(json.dumps(r.get("score"))[:600])
