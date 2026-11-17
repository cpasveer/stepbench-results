import math, sys

def solve(K, f):
    n = len(f)
    A = [row[:] + [f[i]] for i, row in enumerate(K)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(A[r][c]))
        A[c], A[p] = A[p], A[c]
        piv = A[c][c]
        for r in range(c + 1, n):
            m = A[r][c] / piv
            if m != 0.0:
                for k in range(c, n + 1):
                    A[r][k] -= m * A[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = A[r][n] - sum(A[r][k] * x[k] for k in range(r + 1, n))
        x[r] = s / A[r][r]
    return x

def sub(a, b): return [a[i] - b[i] for i in range(3)]
def dot(a, b): return sum(a[i] * b[i] for i in range(3))
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def nrm(a): return math.sqrt(dot(a, a))

def frame_k(E, G, A, Iy, Iz, J, L):
    k = [[0.0]*12 for _ in range(12)]
    def s(i, j, v): k[i][j] += v
    ea = E*A/L; gj = G*J/L
    s(0,0,ea); s(0,6,-ea); s(6,0,-ea); s(6,6,ea)
    s(3,3,gj); s(3,9,-gj); s(9,3,-gj); s(9,9,gj)
    # bending in xy plane (uses Iz): dofs v1=1, rz1=5, v2=7, rz2=11
    a = 12*E*Iz/L**3; b = 6*E*Iz/L**2; c = 4*E*Iz/L; d = 2*E*Iz/L
    idx = [1,5,7,11]
    kk = [[a,b,-a,b],[b,c,-b,d],[-a,-b,a,-b],[b,d,-b,c]]
    for i in range(4):
        for j in range(4): s(idx[i], idx[j], kk[i][j])
    # bending in xz plane (uses Iy): dofs w1=2, ry1=4, w2=8, ry2=10
    a = 12*E*Iy/L**3; b = 6*E*Iy/L**2; c = 4*E*Iy/L; d = 2*E*Iy/L
    idx = [2,4,8,10]
    kk = [[a,-b,-a,-b],[-b,c,b,d],[-a,b,a,b],[-b,d,b,c]]
    for i in range(4):
        for j in range(4): s(idx[i], idx[j], kk[i][j])
    return k

def transform(xa, ref):
    # local x = xa; local y, z from reference vector
    ex = xa
    ey = cross(ref, ex)
    if nrm(ey) < 1e-6:
        ey = cross([0,0,1], ex)
    ey = [v/nrm(ey) for v in ey]
    ez = cross(ex, ey)
    return [ex, ey, ez]

def build_T(R):
    T = [[0.0]*12 for _ in range(12)]
    for blk in range(4):
        for i in range(3):
            for j in range(3):
                T[blk*3+i][blk*3+j] = R[i][j]
    return T

def mat_mul(A, B):
    n, m, p = len(A), len(B), len(B[0])
    return [[sum(A[i][k]*B[k][j] for k in range(m)) for j in range(p)] for i in range(n)]

def T_t(T): return [list(r) for r in zip(*T)]

def deflection(path_pts, E, G, a, t, load_dir, nsub=6):
    """Cantilever: fixed at last point, load at first point. Return k = F/delta along load_dir."""
    Ai = (a*a) - (a-2*t)**2
    I = (a**4 - (a-2*t)**4)/12.0
    J = 2*t*(a-t)**2
    # discretise
    nodes = []
    for p0, p1 in zip(path_pts[:-1], path_pts[1:]):
        L = nrm(sub(p1, p0))
        for i in range(nsub):
            nodes.append([p0[k] + (p1[k]-p0[k])*i/nsub for k in range(3)])
    nodes.append(path_pts[-1])
    N = len(nodes)
    ndof = 6*N
    K = [[0.0]*ndof for _ in range(ndof)]
    for e in range(N-1):
        p0, p1 = nodes[e], nodes[e+1]
        xa = [v/nrm(sub(p1,p0)) for v in sub(p1,p0)]
        R = transform(xa, [0,0,1] if abs(xa[2]) < 0.9 else [0,1,0])
        L = nrm(sub(p1,p0))
        kl = frame_k(E, G, Ai, I, I, J, L)
        T = build_T([R[0], R[1], R[2]])
        # 12x12 transform: block diag of R (rows = local axes)
        Tm = [[0.0]*12 for _ in range(12)]
        for blk in range(4):
            for i in range(3):
                for j in range(3):
                    Tm[blk*3+i][blk*3+j] = R[i][j]
        kg = mat_mul(T_t(Tm), mat_mul(kl, Tm))
        dofs = list(range(6*e, 6*e+6)) + list(range(6*(e+1), 6*(e+1)+6))
        for i in range(12):
            for j in range(12):
                K[dofs[i]][dofs[j]] += kg[i][j]
    # fix last node (A side)
    free = list(range(0, ndof - 6))
    Kf = [[K[i][j] for j in free] for i in free]
    f = [0.0]*len(free)
    d = load_dir
    for k in range(3):
        f[k] = d[k]
    u = solve(Kf, f)
    disp = sum(u[k]*d[k] for k in range(3))
    return 1.0/disp

def estimate(E, path, a, t, n=1):
    G = E/2.6
    best = None
    for ld in ([1,0,0],[0,1,0],[0,0,1]):
        k = deflection(path, E, G, a, t, ld)
        best = k if best is None else min(best, k)
    return best

if __name__ == "__main__":
    pass
