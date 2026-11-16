import sys, numpy as np, math
sys.path.insert(0, "scratch")
from frame import *
A_PADS = [(30,0),(15,26),(-15,26),(-30,0),(-15,-26),(15,-26)]
A_PADS = [(x,y) for x,y in A_PADS]
def tob(s,v,h=0.0): return cB + s*tB + np.array([0,v,0]) + h*nB
B_PADS = [(-15,-40),(15,-40),(-15,40),(15,40)]
def truss(E, T, sA, sB, a_idx=range(6), b_idx=range(4)):
    m = Model(); M = m.node(cB)
    Tn = m.node(T)
    for k in a_idx:
        p = A_PADS[k]; n = m.node([p[0], p[1], 0.0])
        m.nodes  # placeholder
        m.beam(n, Tn, sA*sA, sA**4/12, sA**4/12, 0.14*sA**4, E)
        m.elems[-1] = m.elems[-1]  # keep
        m.nodes[n] = m.nodes[n]
        m.fixed = getattr(m, "fixed", []) + [n]
    for k in b_idx:
        s,v = B_PADS[k]; n = m.node(tob(s,v))
        rigid_link(m, n, M)
        m.beam(n, Tn, sB*sB, sB**4/12, sB**4/12, 0.14*sB**4, E)
    return m, M, m.fixed

def stiff(E, T, sA, sB, **kw):
    out=[]
    for d in ([1,0,0],[0,1,0],[0,0,1]):
        m, M, fixed = truss(E, T, sA, sB, **kw)
        u = m.solve(fixed, M, np.array([*d,0,0,0.]))
        out.append(round(float(1/abs(np.dot(u[6*M:6*M+3], d))),1))
    return out
if __name__ == "__main__":
    for T in ([10,-12,40.0],[10,-12,30.0],[0,0,20.0]):
        for sA,sB in [(4,4),(5,5),(6,6)]:
            print("T",T,"sA",sA,"sB",sB, stiff(1000, np.array(T), sA, sB))
