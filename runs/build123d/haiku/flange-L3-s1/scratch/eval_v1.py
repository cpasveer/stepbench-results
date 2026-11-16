import sys, numpy as np
sys.path.insert(0, "scratch")
from frame import *

def tob(s, v, h=0.0):   # B-local (s along tB, v along y, h along nB) -> A-local
    return cB + s*tB + np.array([0, v, 0]) + h*nB

def build(E, tie_top, arm=13.0, bar=(4.5, 5.0), n_arm=4, pad_only_tie=True):
    m = Model()
    A_root = m.node([0, 0, 0])
    fixed = [A_root]
    # arm from root to top, n_arm segments
    M = m.node(cB)
    prev = A_root
    for k in range(1, n_arm+1):
        if k == n_arm and tie_top:
            cur = M
        else:
            cur = m.node(cB * k / n_arm)
        m.beam(prev, cur, arm*arm, arm**4/12, arm**4/12, 0.1406*arm**4, E)
        prev = cur
    top = prev
    # B-hub nodes
    pads = {}
    for (s, v) in [(-15,-40),(15,-40),(-15,40),(15,40)]:
        pads[(s,v)] = m.node(tob(s, v))
        rigid_link(m, pads[(s,v)], M)
    cross_c = top  # arm top = crossbar centre
    grid = {}
    for s in (-15, 15):
        for v in (-40, 0, 40):
            if v == 0 and s == 0: continue
            grid[(s,v)] = pads.get((s,v)) if v != 0 else m.node(tob(s, 0))
    # vertical bars
    w, t = bar
    A = w*t; I = w*t**3/12; Ib = t*w**3/12; J = 0.25*w*t**3*(1.0)  # approx
    def bm(i, j):
        m.beam(i, j, A, max(I,Ib), max(I,Ib), J, E)
    for s in (-15, 15):
        bm(pads[(s,-40)], grid[(s,0)]); bm(grid[(s,0)], pads[(s,40)])
    # crossbar through the arm top
    bm(grid[(-15,0)], top); bm(top, grid[(15,0)])
    # fix A side
    return m, fixed, M

def stiffness(E, tie_top, **kw):
    out = []
    for d in ([1,0,0],[0,1,0],[0,0,1]):
        m, fixed, M = build(E, tie_top, **kw)
        F6 = np.array([*d, 0,0,0], float)
        u = m.solve(fixed, M, F6)
        disp = u[6*M:6*M+3]
        out.append(1.0 / max(abs(np.dot(disp, d)), 1e-30))
    return out

if __name__ == "__main__":
    for E in (1000, 1500, 3500):
        print("E", E, "tie_top=False", [round(x,2) for x in stiffness(E, False)],
              "tie_top=True", [round(x,2) for x in stiffness(E, True)])

def arm_only(E, tie_top=True, arm=13.0):
    m = Model(); A_root = m.node([0,0,0]); M = m.node(cB)
    n = 4; prev = A_root
    for k in range(1, n+1):
        cur = M if k == n else m.node(cB*k/n)
        m.beam(prev, cur, arm*arm, arm**4/12, arm**4/12, 0.1406*arm**4, E); prev = cur
    out = []
    for d in ([1,0,0],[0,1,0],[0,0,1]):
        u = m.solve([A_root], M, np.array([*d,0,0,0.]))
        out.append(round(1/abs(np.dot(u[6*M:6*M+3], d)), 2))
    return out
