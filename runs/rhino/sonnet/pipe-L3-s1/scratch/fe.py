import json, sys, time
import numpy as np

H = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
MESH = sys.argv[1] if len(sys.argv) > 1 else 'mesh_local.json'
TASK = json.load(open('./task.json'))
PB = TASK['poses'][0]['placements']['B']; PA = TASK['poses'][0]['placements']['A']
O = np.array(PA['origin']); R = np.array(PA['rotation'])
Bo = R.T @ (np.array(PB['origin']) - O); Rb = np.array(PB['rotation'])
U = R.T @ Rb[:, 0]; V = R.T @ Rb[:, 1]; N = R.T @ Rb[:, 2]

d = json.load(open(MESH)); Vt = np.array(d['v']); F = np.array(d['f'])
lo = Vt.min(0) - 2 * H; hi = Vt.max(0) + 2 * H
n = np.ceil((hi - lo) / H).astype(int)
print('grid', n, 'tris', len(F))
# scanline along x
cols = {}
ys = lo[1] + (np.arange(n[1]) + 0.5) * H
zs = lo[2] + (np.arange(n[2]) + 0.5) * H
hits = [[[] for _ in range(n[2])] for _ in range(n[1])]
for tri in F:
    a, b, c = Vt[tri[0]], Vt[tri[1]], Vt[tri[2]]
    y0, y1 = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
    z0, z1 = min(a[2], b[2], c[2]), max(a[2], b[2], c[2])
    iy0 = max(0, int(np.floor((y0 - lo[1]) / H - 0.5))); iy1 = min(n[1] - 1, int(np.ceil((y1 - lo[1]) / H - 0.5)))
    iz0 = max(0, int(np.floor((z0 - lo[2]) / H - 0.5))); iz1 = min(n[2] - 1, int(np.ceil((z1 - lo[2]) / H - 0.5)))
    if iy1 < iy0 or iz1 < iz0: continue
    det = (b[1] - a[1]) * (c[2] - a[2]) - (c[1] - a[1]) * (b[2] - a[2])
    if abs(det) < 1e-12: continue
    for iy in range(iy0, iy1 + 1):
        py = ys[iy]
        for iz in range(iz0, iz1 + 1):
            pz = zs[iz]
            l1 = ((py - a[1]) * (c[2] - a[2]) - (c[1] - a[1]) * (pz - a[2])) / det
            l2 = ((b[1] - a[1]) * (pz - a[2]) - (py - a[1]) * (b[2] - a[2])) / det
            l0 = 1 - l1 - l2
            if l0 >= 0 and l1 >= 0 and l2 >= 0:
                hits[iy][iz].append(l0 * a[0] + l1 * b[0] + l2 * c[0])
occ = np.zeros(n, bool)
xs = lo[0] + (np.arange(n[0]) + 0.5) * H
for iy in range(n[1]):
    for iz in range(n[2]):
        h = sorted(hits[iy][iz])
        for k in range(0, len(h) - 1, 2):
            m = (xs > h[k]) & (xs < h[k + 1])
            occ[:, iy, iz] |= m
print('solid voxels', occ.sum(), 'volume', occ.sum() * H ** 3)

# nodes / elements
ex, ey, ez = np.nonzero(occ)
nn = (n[0] + 1, n[1] + 1, n[2] + 1)
def nid(i, j, k): return (i * nn[1] + j) * nn[2] + k
corner = [(0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1),(1,1,1),(0,1,1)]
en = np.stack([nid(ex + c[0], ey + c[1], ez + c[2]) for c in corner], 1)
used, inv = np.unique(en.ravel(), return_inverse=True)
en = inv.reshape(en.shape)
nnodes = len(used)
# node coordinates
ui = used // (nn[1] * nn[2]); uj = (used // nn[2]) % nn[1]; uk = used % nn[2]
X = lo + np.stack([ui, uj, uk], 1) * H
print('nodes', nnodes, 'dofs', 3 * nnodes)

# hex8 stiffness (E=1)
nu = 0.35
Dm = (1.0 / ((1 + nu) * (1 - 2 * nu))) * np.array([[1-nu, nu, nu, 0,0,0],[nu,1-nu,nu,0,0,0],[nu,nu,1-nu,0,0,0],[0,0,0,(1-2*nu)/2,0,0],[0,0,0,0,(1-2*nu)/2,0],[0,0,0,0,0,(1-2*nu)/2]])
sgn = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]], float)
g = 1 / np.sqrt(3)
Ke = np.zeros((24, 24))
for gx in (-g, g):
    for gy in (-g, g):
        for gz in (-g, g):
            dN = np.zeros((8, 3))
            for a in range(8):
                dN[a] = [sgn[a,0]*(1+sgn[a,1]*gy)*(1+sgn[a,2]*gz)/8, sgn[a,1]*(1+sgn[a,0]*gx)*(1+sgn[a,2]*gz)/8, sgn[a,2]*(1+sgn[a,0]*gx)*(1+sgn[a,1]*gy)/8]
            dN = dN * (2.0 / H)
            B = np.zeros((6, 24))
            for a in range(8):
                B[0, 3*a] = dN[a,0]; B[1, 3*a+1] = dN[a,1]; B[2, 3*a+2] = dN[a,2]
                B[3, 3*a] = dN[a,1]; B[3, 3*a+1] = dN[a,0]
                B[4, 3*a+1] = dN[a,2]; B[4, 3*a+2] = dN[a,1]
                B[5, 3*a] = dN[a,2]; B[5, 3*a+2] = dN[a,0]
            Ke += B.T @ Dm @ B * (H / 2) ** 3
edofs = (en[:, :, None] * 3 + np.arange(3)[None, None, :]).reshape(len(en), 24)
ndof = 3 * nnodes
def Kmul(u):
    ue = u[edofs]
    fe = ue @ Ke  # Ke symmetric
    return np.bincount(edofs.ravel(), weights=fe.ravel(), minlength=ndof)
diagK = np.zeros(ndof)
dKe = np.diag(Ke)
diagK = np.bincount(edofs.ravel(), weights=np.tile(dKe, len(en)), minlength=ndof)

# boundary conditions
r = np.hypot(X[:, 1], X[:, 2])
fixed_nodes = np.nonzero(r < 34.0 + 0.9 * H)[0]
rel = X - Bo
nB = rel @ N; uB = rel @ U; vB = rel @ V
# washer: seat face region
wash_nodes = np.nonzero((nB < 0.5 * H + 0.2) & (np.abs(uB) < 18) & (np.abs(vB) < 18))[0]
print('fixed nodes', len(fixed_nodes), 'washer nodes', len(wash_nodes))
isfixed = np.zeros(ndof, bool)
for k in range(3): isfixed[3 * fixed_nodes + k] = True
# remove washer nodes that are fixed
wash_nodes = np.setdiff1d(wash_nodes, fixed_nodes)
Lscale = 20.0
center = Bo
# rigid map: u_node = t + w x (x - c); q = [tx,ty,tz, wx*L, wy*L, wz*L]
wd = np.concatenate([3 * wash_nodes + k for k in range(3)])
Rmat = np.zeros((len(wash_nodes), 3, 6))
for i, nd in enumerate(wash_nodes):
    p = X[nd] - center
    Rmat[i, :, 0:3] = np.eye(3)
    Rmat[i, :, 3:6] = -np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]]) * 0  # placeholder
    # w x p = [wy pz - wz py, wz px - wx pz, wx py - wy px]
    M = np.array([[0, p[2], -p[1]], [-p[2], 0, p[0]], [p[1], -p[0], 0]])
    Rmat[i, :, 3:6] = M / Lscale
isw = np.zeros(ndof, bool); isw[wd] = True
free = ~isfixed & ~isw
fidx = np.nonzero(free)[0]
nf = len(fidx)
widx = (3 * wash_nodes[:, None] + np.arange(3)[None, :])  # (nw,3)
def scatter(z):
    u = np.zeros(ndof)
    u[fidx] = z[:nf]
    q = z[nf:]
    u[widx.ravel()] = (Rmat @ q).ravel()
    return u
def op(z):
    u = scatter(z)
    f = Kmul(u)
    out = np.empty_like(z)
    out[:nf] = f[fidx]
    fw = f[widx.ravel()].reshape(-1, 3)
    out[nf:] = np.einsum('nij,ni->j', Rmat, fw)
    return out
# preconditioner: Jacobi for free, approx for q
pre = np.ones(nf + 6)
pre[:nf] = 1.0 / diagK[fidx]
dq = np.zeros(6)
for i in range(len(wash_nodes)):
    dd = diagK[widx[i]]
    dq += np.einsum('ij,i->j', Rmat[i] ** 2, dd)
pre[nf:] = 1.0 / (dq * 1.0)
def cg(b, tol=1e-6, maxit=6000):
    x = np.zeros_like(b); rr = b.copy(); zz = pre * rr; p = zz.copy(); rz = rr @ zz; b0 = np.linalg.norm(b)
    for it in range(maxit):
        Ap = op(p); a = rz / (p @ Ap); x += a * p; rr -= a * Ap
        if np.linalg.norm(rr) < tol * b0: break
        zz = pre * rr; rz2 = rr @ zz; p = zz + (rz2 / rz) * p; rz = rz2
    return x, it
C = np.zeros((6, 6))
t0 = time.time()
for i in range(6):
    b = np.zeros(nf + 6); b[nf + i] = 1.0
    x, it = cg(b)
    C[:, i] = x[nf:]
    print('case', i, 'iters', it, 'time', round(time.time() - t0, 1))
# C maps loads (N, N*mm/Lscale... ) to q (mm, rot*Lscale)
# load vector q-dual: forces in N; moments: dual of w*Lscale => moment/Lscale. For unit dual load i>=3 -> moment = Lscale N*mm.
# generalized: load F (N) and M/span (N) ; displacement d (mm) and rot*span
span = 78.15
S = np.diag([1, 1, 1, span / Lscale, span / Lscale, span / Lscale])  # rot*Lscale -> rot*span ... (q3 = w*Lscale)
# moment M = span * f_m => dual load for q (w*Lscale) is M/Lscale = f_m*span/Lscale
Sd = S
Cg = Sd @ C @ Sd  # displacement-like (mm) per unit generalized load (N) , for E=1
lam = np.linalg.eigvalsh((Cg + Cg.T) / 2)
print('compliance eig (E=1):', lam)
print('stiffness N/mm at E=1 (weakest):', 1 / lam.max())
print('C trans diag', np.diag(C)[:3], 'rot diag', np.diag(Cg)[3:])
np.save('C.npy', C)

# ---- energy distribution for each rotational + translational case
cen = X[en].mean(1)
relc = cen - Bo
nC = relc @ N; uC = relc @ U; vC = relc @ V
rC = np.hypot(cen[:, 1], cen[:, 2])
reg = np.full(len(en), 'wall', dtype=object)
reg[(rC < 38.6)] = 'pad/strip'
reg[(uC > -17.5) & (nC > -0.5) & (rC > 38.6)] = 'plate'
reg[(uC <= -17.5) & (uC > -31) & (np.abs(vC) < 18.5) & (nC < 12) & (rC > 38.6)] = 'block'
for i in range(6):
    b = np.zeros(nf + 6); b[nf + i] = 1.0
    x, it = cg(b)
    u = scatter(x)
    ue = u[edofs]
    e = 0.5 * np.einsum('ni,ij,nj->n', ue, Ke, ue)
    tot = e.sum()
    out = {k: round(float(e[reg == k].sum() / tot), 3) for k in ['pad/strip', 'wall', 'block', 'plate']}
    print('case', i, 'energy share', out)

# ---- printed-material estimate (print axis = local x)
def erode_yz(o, steps):
    r = o.copy()
    for _ in range(steps):
        r2 = r.copy()
        r2[:, 1:, :] &= r[:, :-1, :]; r2[:, :-1, :] &= r[:, 1:, :]
        r2[:, :, 1:] &= r[:, :, :-1]; r2[:, :, :-1] &= r[:, :, 1:]
        r = r2
    return r
def erode_x(o, steps):
    r = o.copy()
    for _ in range(steps):
        r2 = r.copy(); r2[1:] &= r[:-1]; r2[:-1] &= r[1:]; r = r2
    return r
sy = max(1, int(round(1.2 / H))); sx = max(1, int(round(0.8 / H)))
core = erode_x(erode_yz(occ, sy), sx)
vol = occ.sum() * H ** 3; vcore = core.sum() * H ** 3
print('volume', round(vol), 'core', round(vcore), 'printed est', round(vol - vcore + 0.2 * vcore))
