import sys, numpy as np, math
sys.path.insert(0, "scratch")
from frame import *
S = math.sqrt(3)/2
A_PADS = [(30,0),(15,30*S),(-15,30*S),(-30,0),(-15,-30*S),(15,-30*S)]
def tob(s,v,h=0.0): return cB + s*tB + np.array([0,v,0]) + h*nB

def build(E, arm=13.0, barA=(4.5,5.0), barB=(4.5,5.0), T_A=5.0, T_B=5.0, arm_root_on_hub=True):
    m = Model(); fixed = []
    # A-hub: pad nodes (fixed, rigid A) at mid-thickness z=T_A/2; ring bars between pads; spoke to arm root
    pads = []
    for p in A_PADS:
        n = m.node([p[0], p[1], T_A/2]); pads.append(n); fixed.append(n)
    root = m.node([0,0,T_A/2])          # arm root node sits in A-hub mid plane
    wA, tA_ = barA
    AA = wA*tA_; IA = wA*tA_**3/12; IAb = tA_*wA**3/12
    for i in range(6):
        m.beam(pads[i], pads[(i+1)%6], AA, max(IA,IAb), max(IA,IAb), 0.25*wA*tA_**3, E)
    m.beam(root, pads[0], AA, max(IA,IAb), max(IA,IAb), 0.25*wA*tA_**3, E)
    m.beam(root, pads[3], AA, max(IA,IAb), max(IA,IAb), 0.25*wA*tA_**3, E)
    # arm from root to B centre
    M = m.node(cB)
    prev = root
    for k in range(1,5):
        cur = M if k == 4 else m.node(cB*k/4 + np.array([0,0,0])*0)
        m.beam(prev, cur, arm*arm, arm**4/12, arm**4/12, 0.1406*arm**4, E); prev = cur
    # B-hub: pads rigidly tied to M; H bars to arm top node (=M)
    wB, tB_ = barB
    AB = wB*tB_; IB = max(wB*tB_**3, tB_*wB**3)/12
    bp = {}
    for (s,v) in [(-15,-40),(15,-40),(-15,40),(15,40)]:
        bp[(s,v)] = m.node(tob(s,v)); rigid_link(m, bp[(s,v)], M)
    mid = {}
    for s in (-15,15):
        mid[(s,0)] = m.node(tob(s,0)); rigid_link(m, mid[(s,0)], M)
    for s in (-15,15):
        m.beam(bp[(s,-40)], mid[(s,0)], AB, IB, IB, 0.25*wB*tB_**3, E)
        m.beam(mid[(s,0)], bp[(s,40)], AB, IB, IB, 0.25*wB*tB_**3, E)
        m.beam(mid[(s,0)], M, AB, IB, IB, 0.25*wB*tB_**3, E)
    return m, fixed, M

def k_dirs(E, **kw):
    out=[]
    for d in ([1,0,0],[0,1,0],[0,0,1]):
        m, fixed, M = build(E, **kw)
        u = m.solve(fixed, M, np.array([*d,0,0,0.]))
        out.append(round(float(1/abs(np.dot(u[6*M:6*M+3], d))),2))
    return out
if __name__ == "__main__":
    print("v1-like, E=1000:", k_dirs(1000))
    print("v1-like, E=3500:", k_dirs(3500))
