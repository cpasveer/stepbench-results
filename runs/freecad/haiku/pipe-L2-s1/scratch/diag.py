exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1/scratch/keys.py").read().split("which = sys.argv")[0])
b = [x for x in BOLTS if x["id"]=="B1"][0]
g = 6.0; k = KEY["M4"]
c = V(*b["position"]); a = V(*b["axis"])
head_top = c + a*(g + HEAD["M4"][1]); tip = head_top - a*k["sd"]
e1 = ortho(a)
names = {"body":body,"fa":fa,"fb":fb}
for o in heads: names[o]=heads[o]
for lean_deg,tilt in [(25.0,0.0),(25.0,90.0),(25.0,180.0),(25.0,270.0),(0.0,0.0)]:
    lam=math.radians(lean_deg); tv=rot(e1,a,math.radians(tilt))
    s0=a*math.cos(lam)+tv*math.sin(lam); e0=ortho(s0) if lean_deg>0 else e1
    e0=e0-s0*e0.dot(s0); e0.normalize()
    for alpha in (0,30):
        s=rot(s0,a,math.radians(alpha)); e=rot(e0,a,math.radians(alpha))
        bend=tip+s*k["short"]
        ks=seg(tip,bend,k["r"]).fuse(seg(bend,bend+e*k["longl"],k["r"]))
        d={n: round(ks.distToShape(s_)[0],2) for n,s_ in names.items()}
        print("lean",lean_deg,"tilt",tilt,"alpha",alpha,"bend",[round(v,1) for v in bend], d)
