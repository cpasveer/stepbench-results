import math, sys, os
HERE = "/home/user/stepbench-runs/tasks-freecad/flange-L2-s1"
sys.path.insert(0, HERE)
import FreeCAD as App, Part
ns = {"__name__": "design", "__file__": "/home/user/stepbench-runs/tasks-freecad/flange-L2-s1/submission.py"}
exec(open(os.path.join(HERE, "submission.py")).read(), ns)
body = ns["build"]().removeSplitter()
V = App.Vector
BOLTS = ns["BOLTS"]; FACES = ns["FACES"]
ORDER = "sweep"

def key_params(b):
    if b["size"] == "M5":
        return dict(head_h=5.0, head_d=8.5, t=2.5, Ls=25.0, Rl=72.0, pad=ns["T_A"])
    return dict(head_h=6.0, head_d=10.0, t=3.0, Ls=28.0, Rl=82.0, pad=ns["T_B"])

if ORDER == "sweep":
    print("bolt  band_s  blocked_at_angles(deg, step 3) -> free sectors (60 deg)")
    for bid, b in BOLTS.items():
        kp = key_params(b)
        n = V(*b["axis"]).normalize(); p = V(*b["position"])
        # head envelope
        env = Part.makeCylinder(kp["head_d"]/2 + 0.5, kp["head_h"] + 1.0 + 0.0, p + n*kp["pad"], n)
        env_c = body.common(env).Volume
        s_b = kp["pad"] + kp["head_h"] - kp["t"] + kp["Ls"]
        # reference direction: perpendicular basis
        u = V(0,0,1).cross(n)
        if u.Length < 1e-6: u = V(1,0,0)
        u.normalize()
        blocked = []
        for th in range(0, 360, 3):
            base = p + n*(s_b - 3.0)
            sec = Part.makeCylinder(kp["Rl"], 6.0, base, n, 60.0)
            # sector starts along +u; rotate to start angle th
            sec.rotate(p, n, th)
            vol = body.common(sec).Volume
            blocked.append(vol > 0.5)
        free = []
        # contiguous free 60-degree windows
        for start in range(0, 360, 3):
            ok = all(not blocked[((start + k) // 3) % 120] for k in range(0, 61, 3))
            if ok: free.append(start)
        print(bid, round(s_b, 2), "head_envelope_vol=%.3f" % env_c,
              "free_starts=", (free[0], free[-1]) if free else None, "n_free=", len(free))
        # sanity: fraction blocked
        print("   blocked fraction %.2f" % (sum(blocked)/len(blocked)))
