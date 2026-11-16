import sys, numpy as np, math
sys.path.insert(0, "scratch")
from frame import *
import model_v1full as base
S = math.sqrt(3)/2
A_PADS = [(30,0),(15,30*S),(-15,30*S),(-30,0),(-15,-30*S),(15,-30*S)]
def tob(s,v,h=0.0): return cB + s*tB + np.array([0,v,0]) + h*nB

def build(E, arm=13.0, barA=(4.5,5.0), barS=(4.5,5.0), barB=(4.5,5.0), T_A=5.0, spokes="plus", arm_end="M"):
    m = Model(); fixed = []
    pads = []
    for p in A_PADS:
        n = m.node([p[0], p[1], T_A/2]); pads.append(n); fixed.append(n)
    root = m.node([0,0,T_A/2])
    wA, tA_ = barA; AA = wA*tA_; IAa = wA*tA_**3/12; IAb = tA_*wA**3/12; JA = 0.25*wA*tA_**3
    for i in range(6):
        m.beam(pads[i], pads[(i+1)%6], AA, max(IAa,IAb), max(IAa,IAb), JA, E)
    wS, tS = barS; AS = wS*tS; IS = max(wS*tS**3, tS*wS**3)/12; JS = 0.25*wS*tS**3
    targets = [pads[0], pads[3]]
    if spokes == "plus":
        # spokes from root to the midpoints of top/bottom ring bars: make mid nodes
        mt = m.node([0, 30*S, T_A/2]); mb = m.node([0, -30*S, T_A/2])
        # top bar between pad1 (15,26) and pad2 (-15,26) is ring edge (1,2); attach mid-nodes by splitting
        m.beam(pads[1], mt, AA, max(IAa,IAb), max(IAa,IAb), JA, E); m.beam(mt, pads[2], AA, max(IAa,IAb), max(IAa,IAb), JA, E)
        m.beam(pads[4], mb, AA, max(IAa,IAb), max(IAa,IAb), JA, E); m.beam(mb, pads[5], AA, max(IAa,IAb), max(IAa,IAb), JA, E)
        for t in [pads[0], pads[3], mt, mb]:
            m.beam(root, t, AS, IS, IS, JS, E)
    else:
        for t in targets: m.beam(root, t, AS, IS, IS, JS, E)
    M = m.node(cB); prev = root
    for k in range(1,5):
        cur = M if k == 4 else m.node(cB*k/4)
        m.beam(prev, cur, arm*arm, arm**4/12, arm**4/12, 0.1406*arm**4, E); prev = cur
    wB, tB_ = barB; AB = wB*tB_; IB = max(wB*tB_**3, tB_*wB**3)/12
    bp = {}
    for (s,v) in [(-15,-40),(15,-40),(-15,40),(15,40)]:
        bp[(s,v)] = m.node(tob(s,v)); rigid_link(m, bp[(s,v)], M)
    for s in (-15,15):
        mid_ = m.node(tob(s,0)); rigid_link(m, mid_, M)
        m.beam(bp[(s,-40)], mid_, AB, IB, IB, 0.25*wB*tB_**3, E)
        m.beam(mid_, bp[(s,40)], AB, IB, IB, 0.25*wB*tB_**3, E)
    return m, fixed, M

def k_dirs(E, **kw):
    out=[]
    for d in ([1,0,0],[0,1,0],[0,0,1]):
        m, fixed, M = build(E, **kw)
        u = m.solve(fixed, M, np.array([*d,0,0,0.]))
        out.append(round(float(1/abs(np.dot(u[6*M:6*M+3], d))),2))
    return out
if __name__ == "__main__":
    E=3500
    for name, kw in [("v1 (single spoke)", dict(spokes="single")),
                     ("plus spokes 4.5x5", dict(spokes="plus")),
                     ("plus spokes 6x5", dict(spokes="plus", barS=(6,5))),
                     ("plus spokes 8x5", dict(spokes="plus", barS=(8,5))),
                     ("plus 8x5 + ring 6x5", dict(spokes="plus", barS=(8,5), barA=(6,5))),
                     ("plus 6x5 + ring 6x5 + arm16", dict(spokes="plus", barS=(6,5), barA=(6,5), arm=16)),
                    ]:
        print(f"{name:30s}", k_dirs(E, **kw))
